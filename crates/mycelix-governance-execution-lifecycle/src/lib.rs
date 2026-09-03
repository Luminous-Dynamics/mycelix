// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Append-only execution lifecycle for binding governance.
//!
//! This crate models execution state as a deterministic projection over verified,
//! immutable events. Mutable status fields, DHT arrival order, update-array order,
//! timestamps, reputation, Phi, stake, or model output never choose authoritative
//! execution state.
//!
//! The critical crash-safety rule is that `Claimed` is a durable pre-effect fence.
//! A host must commit that claim in one successful transaction before entering a
//! later call that may perform external effects. Once claimed, automatic re-claim
//! and replay are forbidden in v0.1. If completion is ambiguous, the lifecycle
//! becomes `Uncertain` and requires explicit reconciliation.

use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-execution-lifecycle-v0.1";
pub const DOMAIN_PROFILE: &str = "mycelix-governance-execution-domain-v1-blake3-framed";
pub const EVENT_PROFILE: &str = "mycelix-governance-execution-event-v1-blake3-framed";
pub const ATTEMPT_PROFILE: &str = "mycelix-governance-execution-attempt-v1-blake3-framed";
pub const IDEMPOTENCY_PROFILE: &str =
    "mycelix-governance-execution-idempotency-v1-blake3-framed";

const DOMAIN_DOMAIN: &[u8] = b"mycelix/governance/execution-domain/v1";
const DOMAIN_EVENT: &[u8] = b"mycelix/governance/execution-event/v1";
const DOMAIN_ATTEMPT: &[u8] = b"mycelix/governance/execution-attempt/v1";
const DOMAIN_IDEMPOTENCY: &[u8] = b"mycelix/governance/execution-idempotency/v1";
const MAX_TEXT_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

type DigestKey = [u8; 32];

fn digest_key(digest: Digest32) -> DigestKey {
    digest.0
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProfiledDigest {
    pub digest: Digest32,
    pub profile: String,
}

impl ProfiledDigest {
    pub fn validate(&self, field: &'static str) -> Result<(), LifecycleError> {
        require_digest(self.digest, field)?;
        validate_profile(&self.profile, field)
    }
}

/// Everything that must remain identical for one real-world execution domain.
///
/// A claim for this domain cannot be reused when the constitution, proposal
/// authority, binding tally, threshold authorization, executable action bytes,
/// or effect-safety policy changes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionDomain {
    pub protocol_version: String,
    pub proposal_id: String,
    pub timelock_ref: String,
    pub proposal_authority_ref: String,
    pub constitutional_epoch: ProfiledDigest,
    pub actions: ProfiledDigest,
    pub binding_tally_ref: String,
    pub threshold_authorization_ref: String,
    /// Commitment to the exact runtime safety policy used for external effects.
    /// Examples include a profile requiring downstream idempotency or a trusted
    /// single-writer fencing service. The kernel binds the policy but does not
    /// implement the external mechanism itself.
    pub effect_safety_policy: ProfiledDigest,
}

impl ExecutionDomain {
    pub fn validate(&self) -> Result<(), LifecycleError> {
        require_protocol(&self.protocol_version)?;
        validate_text(&self.proposal_id, "domain.proposal_id")?;
        validate_text(&self.timelock_ref, "domain.timelock_ref")?;
        validate_text(
            &self.proposal_authority_ref,
            "domain.proposal_authority_ref",
        )?;
        self.constitutional_epoch
            .validate("domain.constitutional_epoch")?;
        self.actions.validate("domain.actions")?;
        validate_text(&self.binding_tally_ref, "domain.binding_tally_ref")?;
        validate_text(
            &self.threshold_authorization_ref,
            "domain.threshold_authorization_ref",
        )?;
        self.effect_safety_policy
            .validate("domain.effect_safety_policy")
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, LifecycleError> {
        self.validate()?;
        let mut out = Vec::new();
        put_domain(&mut out, DOMAIN_DOMAIN);
        put_str(&mut out, &self.protocol_version);
        put_str(&mut out, &self.proposal_id);
        put_str(&mut out, &self.timelock_ref);
        put_str(&mut out, &self.proposal_authority_ref);
        put_profiled_digest(&mut out, &self.constitutional_epoch);
        put_profiled_digest(&mut out, &self.actions);
        put_str(&mut out, &self.binding_tally_ref);
        put_str(&mut out, &self.threshold_authorization_ref);
        put_profiled_digest(&mut out, &self.effect_safety_policy);
        Ok(out)
    }

    pub fn digest(&self) -> Result<Digest32, LifecycleError> {
        Ok(hash(&self.canonical_bytes()?))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TerminalFailureKind {
    /// Positive evidence says no external effect occurred. v0.1 still never
    /// retries automatically; a later governed protocol may authorize retry.
    NoEffectObserved,
    /// An external effect may have occurred. Automatic retry is forbidden.
    EffectMayHaveOccurred,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleEventKind {
    /// Exact execution domain registered for lifecycle tracking.
    Registered,
    /// Current constitutional/tally/threshold authority admitted execution.
    ReadyAuthorized { preflight_ref: String },
    /// Durable pre-effect fence. Competing claims from the same exact Ready event
    /// intentionally share one deterministic attempt ID even when publication
    /// metadata or claim nonce differs.
    Claimed { claim_nonce: Digest32 },
    /// All externally relevant effects completed with durable evidence.
    Completed {
        attempt_id: Digest32,
        receipt_ref: String,
    },
    /// Execution terminated with an explicit failure classification.
    Failed {
        attempt_id: Digest32,
        failure_kind: TerminalFailureKind,
        evidence_ref: String,
    },
    /// Crash/network/process ambiguity means an effect may or may not have occurred.
    Uncertain {
        attempt_id: Digest32,
        evidence_ref: String,
    },
    /// Governance explicitly cancelled before an execution claim existed.
    Cancelled {
        authorization_ref: String,
        reason_digest: Digest32,
    },
}

impl LifecycleEventKind {
    fn validate(&self) -> Result<(), LifecycleError> {
        match self {
            Self::Registered => Ok(()),
            Self::ReadyAuthorized { preflight_ref } => {
                validate_text(preflight_ref, "event.ready.preflight_ref")
            }
            Self::Claimed { claim_nonce } => require_digest(*claim_nonce, "event.claim_nonce"),
            Self::Completed {
                attempt_id,
                receipt_ref,
            } => {
                require_digest(*attempt_id, "event.completed.attempt_id")?;
                validate_text(receipt_ref, "event.completed.receipt_ref")
            }
            Self::Failed {
                attempt_id,
                evidence_ref,
                ..
            } => {
                require_digest(*attempt_id, "event.failed.attempt_id")?;
                validate_text(evidence_ref, "event.failed.evidence_ref")
            }
            Self::Uncertain {
                attempt_id,
                evidence_ref,
            } => {
                require_digest(*attempt_id, "event.uncertain.attempt_id")?;
                validate_text(evidence_ref, "event.uncertain.evidence_ref")
            }
            Self::Cancelled {
                authorization_ref,
                reason_digest,
            } => {
                validate_text(authorization_ref, "event.cancelled.authorization_ref")?;
                require_digest(*reason_digest, "event.cancelled.reason_digest")
            }
        }
    }

    fn tag(&self) -> u8 {
        match self {
            Self::Registered => 0,
            Self::ReadyAuthorized { .. } => 1,
            Self::Claimed { .. } => 2,
            Self::Completed { .. } => 3,
            Self::Failed { .. } => 4,
            Self::Uncertain { .. } => 5,
            Self::Cancelled { .. } => 6,
        }
    }
}

/// Semantic immutable lifecycle event.
///
/// `actor_id` identifies the institutional actor whose authority the host
/// verified. A persistence adapter should store publishing provenance separately;
/// publication alone must never create lifecycle authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleEvent {
    pub protocol_version: String,
    pub execution_domain_digest: Digest32,
    pub parent_event_id: Option<Digest32>,
    pub actor_id: String,
    pub occurred_at_ms: u64,
    pub kind: LifecycleEventKind,
}

impl LifecycleEvent {
    pub fn validate(&self) -> Result<(), LifecycleError> {
        require_protocol(&self.protocol_version)?;
        require_digest(
            self.execution_domain_digest,
            "event.execution_domain_digest",
        )?;
        if let Some(parent) = self.parent_event_id {
            require_digest(parent, "event.parent_event_id")?;
        }
        validate_text(&self.actor_id, "event.actor_id")?;
        if self.occurred_at_ms == 0 {
            return Err(LifecycleError::ZeroTimestamp);
        }
        self.kind.validate()
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, LifecycleError> {
        self.validate()?;
        let mut out = Vec::new();
        put_domain(&mut out, DOMAIN_EVENT);
        put_str(&mut out, &self.protocol_version);
        put_digest(&mut out, self.execution_domain_digest);
        put_optional_digest(&mut out, self.parent_event_id);
        put_str(&mut out, &self.actor_id);
        put_u64(&mut out, self.occurred_at_ms);
        put_u8(&mut out, self.kind.tag());
        match &self.kind {
            LifecycleEventKind::Registered => {}
            LifecycleEventKind::ReadyAuthorized { preflight_ref } => {
                put_str(&mut out, preflight_ref);
            }
            LifecycleEventKind::Claimed { claim_nonce } => {
                put_digest(&mut out, *claim_nonce);
            }
            LifecycleEventKind::Completed {
                attempt_id,
                receipt_ref,
            } => {
                put_digest(&mut out, *attempt_id);
                put_str(&mut out, receipt_ref);
            }
            LifecycleEventKind::Failed {
                attempt_id,
                failure_kind,
                evidence_ref,
            } => {
                put_digest(&mut out, *attempt_id);
                put_u8(
                    &mut out,
                    match failure_kind {
                        TerminalFailureKind::NoEffectObserved => 0,
                        TerminalFailureKind::EffectMayHaveOccurred => 1,
                    },
                );
                put_str(&mut out, evidence_ref);
            }
            LifecycleEventKind::Uncertain {
                attempt_id,
                evidence_ref,
            } => {
                put_digest(&mut out, *attempt_id);
                put_str(&mut out, evidence_ref);
            }
            LifecycleEventKind::Cancelled {
                authorization_ref,
                reason_digest,
            } => {
                put_str(&mut out, authorization_ref);
                put_digest(&mut out, *reason_digest);
            }
        }
        Ok(out)
    }

    pub fn id(&self) -> Result<Digest32, LifecycleError> {
        Ok(hash(&self.canonical_bytes()?))
    }
}

/// Host-created verification wrapper. The kernel never infers authority from
/// event contents themselves.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedLifecycleEvent {
    pub event: LifecycleEvent,
    pub verification_ref: String,
}

impl VerifiedLifecycleEvent {
    pub fn validate_for(&self, domain: &ExecutionDomain) -> Result<Digest32, LifecycleError> {
        self.event.validate()?;
        validate_text(&self.verification_ref, "verified.verification_ref")?;
        let domain_digest = domain.digest()?;
        if self.event.execution_domain_digest != domain_digest {
            return Err(LifecycleError::WrongExecutionDomain);
        }
        self.event.id()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleState {
    Unregistered,
    Registered { event_id: Digest32 },
    Ready { event_id: Digest32 },
    Claimed {
        event_id: Digest32,
        attempt_id: Digest32,
    },
    Completed {
        event_id: Digest32,
        attempt_id: Digest32,
        receipt_ref: String,
    },
    Failed {
        event_id: Digest32,
        attempt_id: Digest32,
        failure_kind: TerminalFailureKind,
        evidence_ref: String,
    },
    Uncertain {
        event_id: Digest32,
        attempt_id: Digest32,
        evidence_ref: String,
    },
    Cancelled { event_id: Digest32 },
}

impl LifecycleState {
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            Self::Completed { .. }
                | Self::Failed { .. }
                | Self::Uncertain { .. }
                | Self::Cancelled { .. }
        )
    }

    /// Only a uniquely projected Ready state may enter the durable claim step.
    pub fn automatic_execution_allowed(&self) -> bool {
        matches!(self, Self::Ready { .. })
    }
}

/// Deterministic attempt identity for one exact execution domain and one exact
/// Ready authorization event.
///
/// Unlike the claim event ID, this intentionally excludes claim publisher,
/// timestamp, and nonce. Two concurrent claims racing from the same exact Ready
/// event therefore derive the same downstream idempotency domain even though the
/// lifecycle projector still treats the competing claim events as a fork.
pub fn execution_attempt_id(
    domain: &ExecutionDomain,
    ready_event_id: Digest32,
) -> Result<Digest32, LifecycleError> {
    let domain_digest = domain.digest()?;
    attempt_id_from_digest(domain_digest, ready_event_id)
}

fn attempt_id_from_digest(
    execution_domain_digest: Digest32,
    ready_event_id: Digest32,
) -> Result<Digest32, LifecycleError> {
    require_digest(
        execution_domain_digest,
        "attempt.execution_domain_digest",
    )?;
    require_digest(ready_event_id, "attempt.ready_event_id")?;
    let mut out = Vec::new();
    put_domain(&mut out, DOMAIN_ATTEMPT);
    put_digest(&mut out, execution_domain_digest);
    put_digest(&mut out, ready_event_id);
    Ok(hash(&out))
}

/// Deterministically project the complete verified event set.
///
/// Security properties:
/// - exact duplicate semantic events collapse by event ID;
/// - every non-root event must name a present parent;
/// - exactly one root may exist and it must be `Registered`;
/// - every verified event must be reachable from that root;
/// - child timestamps may not predate their parent, but timestamps never select
///   among competing children;
/// - two distinct verified children of one parent are an explicit ambiguity.
pub fn project_lifecycle(
    domain: &ExecutionDomain,
    events: &[VerifiedLifecycleEvent],
) -> Result<LifecycleState, LifecycleError> {
    domain.validate()?;
    let domain_digest = domain.digest()?;

    let mut unique: BTreeMap<DigestKey, (Digest32, &VerifiedLifecycleEvent)> = BTreeMap::new();
    for verified in events {
        let id = verified.validate_for(domain)?;
        let key = digest_key(id);
        if let Some((_, existing)) = unique.get(&key) {
            if existing.event != verified.event {
                return Err(LifecycleError::DigestCollisionOrConflictingDuplicate);
            }
            continue;
        }
        unique.insert(key, (id, verified));
    }

    if unique.is_empty() {
        return Ok(LifecycleState::Unregistered);
    }

    // Fail closed on a verified event set that references data we do not have.
    for (id, verified) in unique.values() {
        if let Some(parent) = verified.event.parent_event_id {
            if !unique.contains_key(&digest_key(parent)) {
                return Err(LifecycleError::OrphanParent {
                    event: *id,
                    parent,
                });
            }
        }
    }

    let roots: Vec<(Digest32, &VerifiedLifecycleEvent)> = unique
        .values()
        .filter_map(|(id, event)| {
            if event.event.parent_event_id.is_none() {
                Some((*id, *event))
            } else {
                None
            }
        })
        .collect();

    if roots.is_empty() {
        return Err(LifecycleError::NoLifecycleRoot);
    }
    if roots.len() != 1 {
        return Err(LifecycleError::AmbiguousLifecycleFork {
            parent: None,
            children: sorted_ids(roots.iter().map(|(id, _)| *id)),
        });
    }

    let (mut current_id, root) = roots[0];
    if !matches!(root.event.kind, LifecycleEventKind::Registered) {
        return Err(LifecycleError::InvalidRootEvent);
    }

    let mut current_at_ms = root.event.occurred_at_ms;
    let mut state = LifecycleState::Registered {
        event_id: current_id,
    };
    let mut visited: BTreeSet<DigestKey> = BTreeSet::new();
    visited.insert(digest_key(current_id));

    loop {
        let children: Vec<(Digest32, &VerifiedLifecycleEvent)> = unique
            .values()
            .filter_map(|(id, event)| {
                if event.event.parent_event_id == Some(current_id) {
                    Some((*id, *event))
                } else {
                    None
                }
            })
            .collect();

        if children.is_empty() {
            if visited.len() != unique.len() {
                return Err(LifecycleError::UnreachableVerifiedEvent);
            }
            return Ok(state);
        }

        // Timestamp order is intentionally irrelevant here. Competing verified
        // children freeze execution even if one is earlier, later, or observed first.
        if children.len() != 1 {
            return Err(LifecycleError::AmbiguousLifecycleFork {
                parent: Some(current_id),
                children: sorted_ids(children.iter().map(|(id, _)| *id)),
            });
        }

        let (next_id, next) = children[0];
        if next.event.occurred_at_ms < current_at_ms {
            return Err(LifecycleError::CausalTimeRegression {
                parent: current_id,
                child: next_id,
            });
        }
        if !visited.insert(digest_key(next_id)) {
            return Err(LifecycleError::CycleDetected);
        }

        state = apply_transition(domain_digest, &state, next_id, &next.event.kind)?;
        current_id = next_id;
        current_at_ms = next.event.occurred_at_ms;
    }
}

fn sorted_ids(iter: impl Iterator<Item = Digest32>) -> Vec<Digest32> {
    let mut ids: Vec<Digest32> = iter.collect();
    ids.sort_by_key(|id| id.0);
    ids
}

fn apply_transition(
    domain_digest: Digest32,
    state: &LifecycleState,
    event_id: Digest32,
    kind: &LifecycleEventKind,
) -> Result<LifecycleState, LifecycleError> {
    match (state, kind) {
        (LifecycleState::Registered { .. }, LifecycleEventKind::ReadyAuthorized { .. }) => {
            Ok(LifecycleState::Ready { event_id })
        }
        (LifecycleState::Registered { .. }, LifecycleEventKind::Cancelled { .. })
        | (LifecycleState::Ready { .. }, LifecycleEventKind::Cancelled { .. }) => {
            Ok(LifecycleState::Cancelled { event_id })
        }
        (
            LifecycleState::Ready {
                event_id: ready_event_id,
            },
            LifecycleEventKind::Claimed { .. },
        ) => Ok(LifecycleState::Claimed {
            event_id,
            attempt_id: attempt_id_from_digest(domain_digest, *ready_event_id)?,
        }),
        (
            LifecycleState::Claimed { attempt_id, .. },
            LifecycleEventKind::Completed {
                attempt_id: completed_attempt,
                receipt_ref,
            },
        ) if completed_attempt == attempt_id => Ok(LifecycleState::Completed {
            event_id,
            attempt_id: *attempt_id,
            receipt_ref: receipt_ref.clone(),
        }),
        (
            LifecycleState::Claimed { attempt_id, .. },
            LifecycleEventKind::Failed {
                attempt_id: failed_attempt,
                failure_kind,
                evidence_ref,
            },
        ) if failed_attempt == attempt_id => Ok(LifecycleState::Failed {
            event_id,
            attempt_id: *attempt_id,
            failure_kind: failure_kind.clone(),
            evidence_ref: evidence_ref.clone(),
        }),
        (
            LifecycleState::Claimed { attempt_id, .. },
            LifecycleEventKind::Uncertain {
                attempt_id: uncertain_attempt,
                evidence_ref,
            },
        ) if uncertain_attempt == attempt_id => Ok(LifecycleState::Uncertain {
            event_id,
            attempt_id: *attempt_id,
            evidence_ref: evidence_ref.clone(),
        }),
        (state, _) if state.is_terminal() => Err(LifecycleError::TransitionAfterTerminal),
        (LifecycleState::Claimed { .. }, LifecycleEventKind::Completed { .. })
        | (LifecycleState::Claimed { .. }, LifecycleEventKind::Failed { .. })
        | (LifecycleState::Claimed { .. }, LifecycleEventKind::Uncertain { .. }) => {
            Err(LifecycleError::AttemptMismatch)
        }
        _ => Err(LifecycleError::InvalidTransition),
    }
}

/// Deterministic downstream idempotency token for one claimed attempt/action.
/// Hosts should send this to downstream systems that support idempotent writes.
pub fn action_idempotency_key(
    attempt_id: Digest32,
    action_index: u32,
) -> Result<Digest32, LifecycleError> {
    require_digest(attempt_id, "idempotency.attempt_id")?;
    let mut out = Vec::new();
    put_domain(&mut out, DOMAIN_IDEMPOTENCY);
    put_digest(&mut out, attempt_id);
    put_u32(&mut out, action_index);
    Ok(hash(&out))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LifecycleError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidProfile(&'static str),
    ZeroDigest(&'static str),
    ZeroTimestamp,
    WrongExecutionDomain,
    DigestCollisionOrConflictingDuplicate,
    InvalidRootEvent,
    NoLifecycleRoot,
    OrphanParent {
        event: Digest32,
        parent: Digest32,
    },
    UnreachableVerifiedEvent,
    CausalTimeRegression {
        parent: Digest32,
        child: Digest32,
    },
    InvalidTransition,
    TransitionAfterTerminal,
    AttemptMismatch,
    CycleDetected,
    AmbiguousLifecycleFork {
        parent: Option<Digest32>,
        children: Vec<Digest32>,
    },
}

impl fmt::Display for LifecycleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong execution lifecycle protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidProfile(field) => write!(f, "{field} is not a valid profile token"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::ZeroTimestamp => write!(f, "event timestamp must be non-zero"),
            Self::WrongExecutionDomain => write!(f, "event belongs to another execution domain"),
            Self::DigestCollisionOrConflictingDuplicate => {
                write!(f, "same event digest resolves to conflicting semantics")
            }
            Self::InvalidRootEvent => write!(f, "lifecycle root must be Registered"),
            Self::NoLifecycleRoot => write!(f, "verified lifecycle events contain no root"),
            Self::OrphanParent { .. } => write!(f, "verified lifecycle event references a missing parent"),
            Self::UnreachableVerifiedEvent => {
                write!(f, "verified lifecycle set contains data unreachable from its root")
            }
            Self::CausalTimeRegression { .. } => {
                write!(f, "child lifecycle event predates its parent")
            }
            Self::InvalidTransition => write!(f, "invalid execution lifecycle transition"),
            Self::TransitionAfterTerminal => write!(f, "terminal lifecycle state cannot transition"),
            Self::AttemptMismatch => write!(f, "terminal event references another execution attempt"),
            Self::CycleDetected => write!(f, "execution lifecycle contains a cycle"),
            Self::AmbiguousLifecycleFork { .. } => {
                write!(f, "execution lifecycle has competing verified children")
            }
        }
    }
}

impl std::error::Error for LifecycleError {}

fn require_protocol(value: &str) -> Result<(), LifecycleError> {
    if value == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(LifecycleError::WrongProtocolVersion)
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), LifecycleError> {
    if value.trim().is_empty() {
        return Err(LifecycleError::Empty(field));
    }
    if value.len() > MAX_TEXT_BYTES {
        return Err(LifecycleError::TooLong(field));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &'static str) -> Result<(), LifecycleError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(LifecycleError::InvalidProfile(field));
    }
    Ok(())
}

fn require_digest(digest: Digest32, field: &'static str) -> Result<(), LifecycleError> {
    if digest.is_zero() {
        Err(LifecycleError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

fn hash(bytes: &[u8]) -> Digest32 {
    Digest32(*blake3::hash(bytes).as_bytes())
}

fn put_domain(out: &mut Vec<u8>, value: &[u8]) {
    put_bytes(out, value);
}

fn put_profiled_digest(out: &mut Vec<u8>, value: &ProfiledDigest) {
    put_digest(out, value.digest);
    put_str(out, &value.profile);
}

fn put_optional_digest(out: &mut Vec<u8>, value: Option<Digest32>) {
    match value {
        None => put_u8(out, 0),
        Some(digest) => {
            put_u8(out, 1);
            put_digest(out, digest);
        }
    }
}

fn put_digest(out: &mut Vec<u8>, value: Digest32) {
    out.extend_from_slice(&value.0);
}

fn put_str(out: &mut Vec<u8>, value: &str) {
    put_bytes(out, value.as_bytes());
}

fn put_bytes(out: &mut Vec<u8>, value: &[u8]) {
    put_u32(out, value.len() as u32);
    out.extend_from_slice(value);
}

fn put_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn put_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn domain() -> ExecutionDomain {
        ExecutionDomain {
            protocol_version: PROTOCOL_VERSION.into(),
            proposal_id: "MIP-42".into(),
            timelock_ref: "timelock:MIP-42".into(),
            proposal_authority_ref: "uhCkk-authority".into(),
            constitutional_epoch: ProfiledDigest {
                digest: digest(1),
                profile: "mycelix-constitution-statement-v1".into(),
            },
            actions: ProfiledDigest {
                digest: digest(2),
                profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            },
            binding_tally_ref: "uhCkk-tally".into(),
            threshold_authorization_ref: "threshold:sig:1".into(),
            effect_safety_policy: ProfiledDigest {
                digest: digest(3),
                profile: "mycelix-effect-safety-policy-v1".into(),
            },
        }
    }

    fn event(
        domain: &ExecutionDomain,
        parent: Option<Digest32>,
        at: u64,
        kind: LifecycleEventKind,
    ) -> LifecycleEvent {
        LifecycleEvent {
            protocol_version: PROTOCOL_VERSION.into(),
            execution_domain_digest: domain.digest().unwrap(),
            parent_event_id: parent,
            actor_id: "did:mycelix:actor".into(),
            occurred_at_ms: at,
            kind,
        }
    }

    fn verified(event: LifecycleEvent) -> VerifiedLifecycleEvent {
        VerifiedLifecycleEvent {
            event,
            verification_ref: "verified:test".into(),
        }
    }

    fn ready_path() -> (ExecutionDomain, Vec<VerifiedLifecycleEvent>, Digest32) {
        let domain = domain();
        let registered = event(&domain, None, 1, LifecycleEventKind::Registered);
        let registered_id = registered.id().unwrap();
        let ready = event(
            &domain,
            Some(registered_id),
            2,
            LifecycleEventKind::ReadyAuthorized {
                preflight_ref: "preflight:1".into(),
            },
        );
        let ready_id = ready.id().unwrap();
        (
            domain,
            vec![verified(registered), verified(ready)],
            ready_id,
        )
    }

    fn happy_path() -> (
        ExecutionDomain,
        Vec<VerifiedLifecycleEvent>,
        Digest32,
        Digest32,
    ) {
        let (domain, mut events, ready_id) = ready_path();
        let claimed = event(
            &domain,
            Some(ready_id),
            3,
            LifecycleEventKind::Claimed {
                claim_nonce: digest(9),
            },
        );
        let claim_event_id = claimed.id().unwrap();
        let attempt_id = execution_attempt_id(&domain, ready_id).unwrap();
        events.push(verified(claimed));
        (domain, events, claim_event_id, attempt_id)
    }

    #[test]
    fn empty_verified_set_is_unregistered() {
        assert_eq!(
            project_lifecycle(&domain(), &[]).unwrap(),
            LifecycleState::Unregistered
        );
    }

    #[test]
    fn claim_uses_deterministic_attempt_separate_from_claim_event_id() {
        let (domain, events, claim_event_id, attempt_id) = happy_path();
        assert_ne!(claim_event_id, attempt_id);
        assert_eq!(
            project_lifecycle(&domain, &events).unwrap(),
            LifecycleState::Claimed {
                event_id: claim_event_id,
                attempt_id,
            }
        );
    }

    #[test]
    fn competing_claims_share_downstream_attempt_domain_but_still_fork() {
        let (domain, mut events, ready_id) = ready_path();
        let claim_a = event(
            &domain,
            Some(ready_id),
            3,
            LifecycleEventKind::Claimed {
                claim_nonce: digest(10),
            },
        );
        let claim_b = event(
            &domain,
            Some(ready_id),
            300,
            LifecycleEventKind::Claimed {
                claim_nonce: digest(11),
            },
        );
        let attempt_a = execution_attempt_id(&domain, ready_id).unwrap();
        let attempt_b = execution_attempt_id(&domain, ready_id).unwrap();
        assert_eq!(attempt_a, attempt_b);
        assert_eq!(
            action_idempotency_key(attempt_a, 0).unwrap(),
            action_idempotency_key(attempt_b, 0).unwrap()
        );

        events.push(verified(claim_a.clone()));
        events.push(verified(claim_b.clone()));
        let first = project_lifecycle(&domain, &events).unwrap_err();

        let mut reordered = events;
        reordered.reverse();
        let second = project_lifecycle(&domain, &reordered).unwrap_err();
        assert!(matches!(first, LifecycleError::AmbiguousLifecycleFork { .. }));
        assert_eq!(first, second);
    }

    #[test]
    fn completed_must_reference_exact_attempt_and_claim_parent() {
        let (domain, mut events, claim_event_id, attempt_id) = happy_path();
        events.push(verified(event(
            &domain,
            Some(claim_event_id),
            4,
            LifecycleEventKind::Completed {
                attempt_id,
                receipt_ref: "receipt:external:1".into(),
            },
        )));
        assert!(matches!(
            project_lifecycle(&domain, &events).unwrap(),
            LifecycleState::Completed { .. }
        ));

        let mut wrong_events = events[..3].to_vec();
        wrong_events.push(verified(event(
            &domain,
            Some(claim_event_id),
            4,
            LifecycleEventKind::Completed {
                attempt_id: digest(99),
                receipt_ref: "receipt:wrong".into(),
            },
        )));
        assert_eq!(
            project_lifecycle(&domain, &wrong_events).unwrap_err(),
            LifecycleError::AttemptMismatch
        );
    }

    #[test]
    fn competing_terminal_outcomes_fail_closed() {
        let (domain, mut events, claim_event_id, attempt_id) = happy_path();
        events.push(verified(event(
            &domain,
            Some(claim_event_id),
            4,
            LifecycleEventKind::Completed {
                attempt_id,
                receipt_ref: "receipt:1".into(),
            },
        )));
        events.push(verified(event(
            &domain,
            Some(claim_event_id),
            5,
            LifecycleEventKind::Failed {
                attempt_id,
                failure_kind: TerminalFailureKind::EffectMayHaveOccurred,
                evidence_ref: "evidence:failure".into(),
            },
        )));
        assert!(matches!(
            project_lifecycle(&domain, &events).unwrap_err(),
            LifecycleError::AmbiguousLifecycleFork { .. }
        ));
    }

    #[test]
    fn causal_time_regression_is_invalid_but_time_never_selects_forks() {
        let domain = domain();
        let registered = event(&domain, None, 10, LifecycleEventKind::Registered);
        let registered_id = registered.id().unwrap();
        let ready = event(
            &domain,
            Some(registered_id),
            9,
            LifecycleEventKind::ReadyAuthorized {
                preflight_ref: "preflight:1".into(),
            },
        );
        assert!(matches!(
            project_lifecycle(&domain, &[verified(registered), verified(ready)]).unwrap_err(),
            LifecycleError::CausalTimeRegression { .. }
        ));
    }

    #[test]
    fn orphan_verified_event_denies_instead_of_disappearing() {
        let domain = domain();
        let registered = event(&domain, None, 1, LifecycleEventKind::Registered);
        let orphan = event(
            &domain,
            Some(digest(99)),
            2,
            LifecycleEventKind::ReadyAuthorized {
                preflight_ref: "preflight:orphan".into(),
            },
        );
        assert!(matches!(
            project_lifecycle(&domain, &[verified(registered), verified(orphan)]).unwrap_err(),
            LifecycleError::OrphanParent { .. }
        ));
    }

    #[test]
    fn multiple_roots_are_an_explicit_fork() {
        let domain = domain();
        let a = event(&domain, None, 1, LifecycleEventKind::Registered);
        let mut b = event(&domain, None, 1, LifecycleEventKind::Registered);
        b.actor_id = "did:mycelix:other".into();
        assert!(matches!(
            project_lifecycle(&domain, &[verified(a), verified(b)]).unwrap_err(),
            LifecycleError::AmbiguousLifecycleFork { parent: None, .. }
        ));
    }

    #[test]
    fn projection_is_arrival_order_independent() {
        let (domain, mut events, claim_event_id, attempt_id) = happy_path();
        events.push(verified(event(
            &domain,
            Some(claim_event_id),
            4,
            LifecycleEventKind::Completed {
                attempt_id,
                receipt_ref: "receipt:1".into(),
            },
        )));
        let expected = project_lifecycle(&domain, &events).unwrap();
        events.reverse();
        assert_eq!(project_lifecycle(&domain, &events).unwrap(), expected);
    }

    #[test]
    fn exact_duplicate_publication_is_harmless() {
        let (domain, mut events, _, _) = happy_path();
        events.push(events[0].clone());
        assert!(matches!(
            project_lifecycle(&domain, &events).unwrap(),
            LifecycleState::Claimed { .. }
        ));
    }

    #[test]
    fn uncertain_is_terminal_and_never_auto_replays() {
        let (domain, mut events, claim_event_id, attempt_id) = happy_path();
        let uncertain = event(
            &domain,
            Some(claim_event_id),
            4,
            LifecycleEventKind::Uncertain {
                attempt_id,
                evidence_ref: "evidence:crash-window".into(),
            },
        );
        let uncertain_id = uncertain.id().unwrap();
        events.push(verified(uncertain));
        let state = project_lifecycle(&domain, &events).unwrap();
        assert!(matches!(state, LifecycleState::Uncertain { .. }));
        assert!(!state.automatic_execution_allowed());

        events.push(verified(event(
            &domain,
            Some(uncertain_id),
            5,
            LifecycleEventKind::Claimed {
                claim_nonce: digest(44),
            },
        )));
        assert_eq!(
            project_lifecycle(&domain, &events).unwrap_err(),
            LifecycleError::TransitionAfterTerminal
        );
    }

    #[test]
    fn no_effect_failure_is_still_terminal_in_v0_1() {
        let (domain, mut events, claim_event_id, attempt_id) = happy_path();
        events.push(verified(event(
            &domain,
            Some(claim_event_id),
            4,
            LifecycleEventKind::Failed {
                attempt_id,
                failure_kind: TerminalFailureKind::NoEffectObserved,
                evidence_ref: "evidence:no-effect".into(),
            },
        )));
        let state = project_lifecycle(&domain, &events).unwrap();
        assert!(matches!(
            state,
            LifecycleState::Failed {
                failure_kind: TerminalFailureKind::NoEffectObserved,
                ..
            }
        ));
        assert!(!state.automatic_execution_allowed());
    }

    #[test]
    fn changed_execution_domain_invalidates_old_events() {
        let (domain, events, _, _) = happy_path();
        let mut changed = domain.clone();
        changed.actions.digest = digest(77);
        assert_eq!(
            project_lifecycle(&changed, &events).unwrap_err(),
            LifecycleError::WrongExecutionDomain
        );
    }

    #[test]
    fn changed_effect_safety_policy_invalidates_old_claim_domain() {
        let (domain, events, _, _) = happy_path();
        let mut changed = domain.clone();
        changed.effect_safety_policy.digest = digest(88);
        assert_eq!(
            project_lifecycle(&changed, &events).unwrap_err(),
            LifecycleError::WrongExecutionDomain
        );
    }

    #[test]
    fn event_identity_binds_semantics_and_causal_parent() {
        let domain = domain();
        let a = event(&domain, None, 1, LifecycleEventKind::Registered);
        let mut changed_actor = a.clone();
        changed_actor.actor_id = "did:mycelix:other".into();
        let mut changed_time = a.clone();
        changed_time.occurred_at_ms = 2;
        assert_ne!(a.id().unwrap(), changed_actor.id().unwrap());
        assert_ne!(a.id().unwrap(), changed_time.id().unwrap());
    }

    #[test]
    fn idempotency_key_binds_attempt_and_action_index() {
        let attempt = digest(22);
        let a = action_idempotency_key(attempt, 0).unwrap();
        let same = action_idempotency_key(attempt, 0).unwrap();
        let next = action_idempotency_key(attempt, 1).unwrap();
        assert_eq!(a, same);
        assert_ne!(a, next);
        assert_ne!(a, action_idempotency_key(digest(23), 0).unwrap());
    }

    #[test]
    fn cancellation_after_claim_is_not_valid_in_v0_1() {
        let (domain, mut events, claim_event_id, _) = happy_path();
        events.push(verified(event(
            &domain,
            Some(claim_event_id),
            4,
            LifecycleEventKind::Cancelled {
                authorization_ref: "cancel:1".into(),
                reason_digest: digest(88),
            },
        )));
        assert_eq!(
            project_lifecycle(&domain, &events).unwrap_err(),
            LifecycleError::InvalidTransition
        );
    }
}

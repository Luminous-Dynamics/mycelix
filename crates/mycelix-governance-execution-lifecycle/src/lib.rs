// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Append-only execution lifecycle for binding governance.
//!
//! This crate deliberately models lifecycle state as a projection over verified
//! immutable events. A mutable `status` field, DHT arrival order, update-array
//! order, timestamps, reputation, Phi, stake, or model output can never choose
//! the authoritative execution state.
//!
//! The critical crash-safety rule is that `Claimed` is a durable fence. A host
//! must commit the claim in one successful transaction before entering a later
//! call that may perform external effects. Once claimed, automatic re-claim or
//! replay is forbidden in v0.1. If completion is uncertain, the lifecycle ends
//! in `Uncertain` and requires explicit reconciliation outside this protocol.

use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-execution-lifecycle-v0.1";
pub const DOMAIN_PROFILE: &str = "mycelix-governance-execution-domain-v1-blake3-framed";
pub const EVENT_PROFILE: &str = "mycelix-governance-execution-event-v1-blake3-framed";
pub const IDEMPOTENCY_PROFILE: &str =
    "mycelix-governance-execution-idempotency-v1-blake3-framed";

const DOMAIN_DOMAIN: &[u8] = b"mycelix/governance/execution-domain/v1";
const DOMAIN_EVENT: &[u8] = b"mycelix/governance/execution-event/v1";
const DOMAIN_IDEMPOTENCY: &[u8] = b"mycelix/governance/execution-idempotency/v1";
const MAX_TEXT_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

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
/// authority, tally, threshold authorization, or executable action bytes change.
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
        Ok(())
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
        Ok(out)
    }

    pub fn digest(&self) -> Result<Digest32, LifecycleError> {
        Ok(hash(&self.canonical_bytes()?))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TerminalFailureKind {
    /// The verifier has positive evidence that no external effect occurred.
    /// v0.1 still does not retry automatically; a later governed protocol may.
    NoEffectObserved,
    /// An effect may have happened. Automatic retry is forbidden.
    EffectMayHaveOccurred,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleEventKind {
    /// Exact execution domain registered for lifecycle tracking.
    Registered,
    /// Current constitutional/tally/threshold authority has admitted execution.
    ReadyAuthorized {
        preflight_ref: String,
    },
    /// Durable pre-effect fence. This event ID becomes the attempt ID.
    Claimed {
        claim_nonce: Digest32,
    },
    /// All externally relevant effects completed and a receipt/evidence reference exists.
    Completed {
        attempt_id: Digest32,
        receipt_ref: String,
    },
    /// Execution terminated with a known failure classification.
    Failed {
        attempt_id: Digest32,
        failure_kind: TerminalFailureKind,
        evidence_ref: String,
    },
    /// Crash/network/process ambiguity means an effect may or may not have occurred.
    /// This is terminal for automatic execution in v0.1.
    Uncertain {
        attempt_id: Digest32,
        evidence_ref: String,
    },
    /// Governance explicitly cancelled before a claim was created.
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
/// `actor_id` is the institutional actor whose authority was externally
/// verified. A Holochain persistence wrapper should store the publishing author
/// separately; publication alone must not create lifecycle authority.
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

/// Host-created verification wrapper. The lifecycle kernel never infers
/// authority from the event contents themselves.
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
    Registered {
        event_id: Digest32,
    },
    Ready {
        event_id: Digest32,
    },
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
    Cancelled {
        event_id: Digest32,
    },
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

    pub fn automatic_execution_allowed(&self) -> bool {
        matches!(self, Self::Ready { .. })
    }
}

/// Deterministically project verified immutable events.
///
/// Exact duplicate semantic events collapse by event ID. Two distinct verified
/// children of the current parent are never ordered by timestamp or arrival;
/// they are an explicit ambiguity and freeze projection.
pub fn project_lifecycle(
    domain: &ExecutionDomain,
    events: &[VerifiedLifecycleEvent],
) -> Result<LifecycleState, LifecycleError> {
    domain.validate()?;

    let mut unique: BTreeMap<Digest32, &VerifiedLifecycleEvent> = BTreeMap::new();
    for verified in events {
        let id = verified.validate_for(domain)?;
        if let Some(existing) = unique.get(&id) {
            if existing.event != verified.event {
                return Err(LifecycleError::DigestCollisionOrConflictingDuplicate);
            }
            continue;
        }
        unique.insert(id, verified);
    }

    let roots: Vec<(Digest32, &VerifiedLifecycleEvent)> = unique
        .iter()
        .filter_map(|(id, event)| {
            if event.event.parent_event_id.is_none() {
                Some((*id, *event))
            } else {
                None
            }
        })
        .collect();

    if roots.is_empty() {
        return Ok(LifecycleState::Unregistered);
    }
    if roots.len() != 1 {
        return Err(LifecycleError::AmbiguousLifecycleFork {
            parent: None,
            children: roots.iter().map(|(id, _)| *id).collect(),
        });
    }

    let (mut current_id, root) = roots[0];
    if !matches!(root.event.kind, LifecycleEventKind::Registered) {
        return Err(LifecycleError::InvalidRootEvent);
    }
    let mut state = LifecycleState::Registered {
        event_id: current_id,
    };
    let mut visited = BTreeSet::new();
    visited.insert(current_id);

    loop {
        let children: Vec<(Digest32, &VerifiedLifecycleEvent)> = unique
            .iter()
            .filter_map(|(id, event)| {
                if event.event.parent_event_id == Some(current_id) {
                    Some((*id, *event))
                } else {
                    None
                }
            })
            .collect();

        if children.is_empty() {
            return Ok(state);
        }
        if children.len() != 1 {
            return Err(LifecycleError::AmbiguousLifecycleFork {
                parent: Some(current_id),
                children: children.iter().map(|(id, _)| *id).collect(),
            });
        }

        let (next_id, next) = children[0];
        if !visited.insert(next_id) {
            return Err(LifecycleError::CycleDetected);
        }
        state = apply_transition(&state, next_id, &next.event.kind)?;
        current_id = next_id;
    }
}

fn apply_transition(
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
        (LifecycleState::Ready { .. }, LifecycleEventKind::Claimed { .. }) => {
            // The claim event's own canonical ID is the durable attempt ID.
            Ok(LifecycleState::Claimed {
                event_id,
                attempt_id: event_id,
            })
        }
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
///
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
            Self::InvalidTransition => write!(f, "invalid execution lifecycle transition"),
            Self::TransitionAfterTerminal => write!(f, "terminal lifecycle state cannot transition"),
            Self::AttemptMismatch => write!(f, "terminal event references another execution attempt"),
            Self::CycleDetected => write!(f, "execution lifecycle contains a cycle"),
            Self::AmbiguousLifecycleFork { .. } => write!(f, "execution lifecycle has competing verified children"),
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

    fn happy_path() -> (ExecutionDomain, Vec<VerifiedLifecycleEvent>, Digest32) {
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
        let claimed = event(
            &domain,
            Some(ready_id),
            3,
            LifecycleEventKind::Claimed {
                claim_nonce: digest(9),
            },
        );
        let attempt_id = claimed.id().unwrap();
        (
            domain,
            vec![verified(registered), verified(ready), verified(claimed)],
            attempt_id,
        )
    }

    #[test]
    fn claim_event_becomes_durable_attempt_id() {
        let (domain, events, attempt_id) = happy_path();
        assert_eq!(
            project_lifecycle(&domain, &events).unwrap(),
            LifecycleState::Claimed {
                event_id: attempt_id,
                attempt_id,
            }
        );
    }

    #[test]
    fn completed_must_reference_exact_attempt() {
        let (domain, mut events, attempt_id) = happy_path();
        let completed = event(
            &domain,
            Some(attempt_id),
            4,
            LifecycleEventKind::Completed {
                attempt_id,
                receipt_ref: "receipt:external:1".into(),
            },
        );
        events.push(verified(completed));
        assert!(matches!(
            project_lifecycle(&domain, &events).unwrap(),
            LifecycleState::Completed { .. }
        ));

        let mut wrong_events = events[..3].to_vec();
        wrong_events.push(verified(event(
            &domain,
            Some(attempt_id),
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
    fn competing_claims_are_an_explicit_fork() {
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
            3,
            LifecycleEventKind::Claimed {
                claim_nonce: digest(11),
            },
        );
        let err = project_lifecycle(
            &domain,
            &[
                verified(registered),
                verified(ready),
                verified(claim_a),
                verified(claim_b),
            ],
        )
        .unwrap_err();
        assert!(matches!(err, LifecycleError::AmbiguousLifecycleFork { .. }));
    }

    #[test]
    fn projection_is_arrival_order_independent() {
        let (domain, mut events, attempt_id) = happy_path();
        let completed = event(
            &domain,
            Some(attempt_id),
            4,
            LifecycleEventKind::Completed {
                attempt_id,
                receipt_ref: "receipt:1".into(),
            },
        );
        events.push(verified(completed));
        let expected = project_lifecycle(&domain, &events).unwrap();
        events.reverse();
        assert_eq!(project_lifecycle(&domain, &events).unwrap(), expected);
    }

    #[test]
    fn exact_duplicate_publication_is_harmless() {
        let (domain, mut events, _) = happy_path();
        events.push(events[0].clone());
        assert!(matches!(
            project_lifecycle(&domain, &events).unwrap(),
            LifecycleState::Claimed { .. }
        ));
    }

    #[test]
    fn uncertain_is_terminal_and_never_auto_replays() {
        let (domain, mut events, attempt_id) = happy_path();
        let uncertain = event(
            &domain,
            Some(attempt_id),
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
    fn changed_execution_domain_invalidates_old_events() {
        let (domain, events, _) = happy_path();
        let mut changed = domain.clone();
        changed.actions.digest = digest(77);
        assert_eq!(
            project_lifecycle(&changed, &events).unwrap_err(),
            LifecycleError::WrongExecutionDomain
        );
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
        let (domain, mut events, attempt_id) = happy_path();
        events.push(verified(event(
            &domain,
            Some(attempt_id),
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

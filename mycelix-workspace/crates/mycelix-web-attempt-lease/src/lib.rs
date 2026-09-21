#![forbid(unsafe_code)]
//! Pure, zero-I/O WEB-LEASE-001 state-machine semantics.
//!
//! This crate does not persist records, perform compare-and-swap, mint an
//! authoritative lease/start token, open sockets, resolve DNS, perform TLS/HTTP,
//! or call any external system.
//!
//! Its strongest output is a `TransitionPlanV1` that a separately-qualified
//! durable store may attempt to commit atomically.
//!
//! ```text
//! transition plan
//! != durable transition
//! != active lease token
//! != Started authorization
//! != socket authority
//! ```

use std::fmt;

pub const LEASE_PROFILE_V1: &str = "mycelix:web-attempt-lease:v1";
pub const TIME_PROFILE_V1: &str = "mycelix:web-attempt-lease-time:wall-clock-assertion:v1";

pub const MAX_ID_BYTES: usize = 192;
pub const MAX_REF_BYTES: usize = 256;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseAuthorityScopeV1 {
    PureTransitionPlanningOnly,
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub struct AdmissionCommitmentV1([u8; 32]);

impl AdmissionCommitmentV1 {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl fmt::Debug for AdmissionCommitmentV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("AdmissionCommitmentV1")
            .field(&"<redacted-32-bytes>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub struct BoundedId(String);

impl BoundedId {
    pub fn new(value: impl Into<String>) -> Result<Self, LeaseModelError> {
        let value = value.into();
        validate_graphic_ascii("id", &value, MAX_ID_BYTES)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for BoundedId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BoundedId")
            .field("bytes", &self.0.len())
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub struct BoundedRef(String);

impl BoundedRef {
    pub fn new(value: impl Into<String>) -> Result<Self, LeaseModelError> {
        let value = value.into();
        validate_graphic_ascii("reference", &value, MAX_REF_BYTES)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for BoundedRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BoundedRef")
            .field("bytes", &self.0.len())
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub struct AttemptAuthorityKeyV1 {
    admission_commitment: AdmissionCommitmentV1,
    attempt_id: BoundedId,
    execution_profile_id: BoundedId,
}

impl AttemptAuthorityKeyV1 {
    pub fn new(
        admission_commitment: AdmissionCommitmentV1,
        attempt_id: BoundedId,
        execution_profile_id: BoundedId,
    ) -> Self {
        Self {
            admission_commitment,
            attempt_id,
            execution_profile_id,
        }
    }

    pub const fn admission_commitment(&self) -> &AdmissionCommitmentV1 {
        &self.admission_commitment
    }

    pub fn attempt_id(&self) -> &BoundedId {
        &self.attempt_id
    }

    pub fn execution_profile_id(&self) -> &BoundedId {
        &self.execution_profile_id
    }
}

impl fmt::Debug for AttemptAuthorityKeyV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AttemptAuthorityKeyV1")
            .field("admission_commitment", &self.admission_commitment)
            .field("attempt_id", &self.attempt_id)
            .field("execution_profile_id", &self.execution_profile_id)
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct WallClockAssertionMsV1(pub u64);

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LeaseStateV1 {
    Prepared {
        lease_epoch: u64,
    },
    Leased {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        issued_at: WallClockAssertionMsV1,
        expires_at: WallClockAssertionMsV1,
    },
    Started {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        started_at: WallClockAssertionMsV1,
    },
    FinishedSuccess {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        outcome_ref: BoundedRef,
    },
    FinishedFailure {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        outcome_ref: BoundedRef,
    },
    AbortedBeforeStart {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        reason_ref: BoundedRef,
    },
    IndeterminateAfterCrash {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        recovery_ref: BoundedRef,
    },
}

impl LeaseStateV1 {
    pub const fn lease_epoch(&self) -> u64 {
        match self {
            Self::Prepared { lease_epoch }
            | Self::Leased { lease_epoch, .. }
            | Self::Started { lease_epoch, .. }
            | Self::FinishedSuccess { lease_epoch, .. }
            | Self::FinishedFailure { lease_epoch, .. }
            | Self::AbortedBeforeStart { lease_epoch, .. }
            | Self::IndeterminateAfterCrash { lease_epoch, .. } => *lease_epoch,
        }
    }

    pub const fn is_terminal(&self) -> bool {
        matches!(
            self,
            Self::FinishedSuccess { .. }
                | Self::FinishedFailure { .. }
                | Self::AbortedBeforeStart { .. }
                | Self::IndeterminateAfterCrash { .. }
        )
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VersionedAttemptRecordV1 {
    key: AttemptAuthorityKeyV1,
    store_revision: u64,
    state: LeaseStateV1,
}

impl VersionedAttemptRecordV1 {
    pub fn prepared(key: AttemptAuthorityKeyV1) -> Self {
        Self {
            key,
            store_revision: 0,
            state: LeaseStateV1::Prepared { lease_epoch: 0 },
        }
    }

    pub fn from_parts(
        key: AttemptAuthorityKeyV1,
        store_revision: u64,
        state: LeaseStateV1,
    ) -> Self {
        Self {
            key,
            store_revision,
            state,
        }
    }

    pub fn key(&self) -> &AttemptAuthorityKeyV1 {
        &self.key
    }

    pub const fn store_revision(&self) -> u64 {
        self.store_revision
    }

    pub fn state(&self) -> &LeaseStateV1 {
        &self.state
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LeaseOperationV1 {
    EnsurePrepared {
        key: AttemptAuthorityKeyV1,
    },
    AcquireLease {
        lease_id: BoundedId,
        holder_id: BoundedId,
        issued_at: WallClockAssertionMsV1,
        expires_at: WallClockAssertionMsV1,
    },
    ReacquireExpiredLease {
        observed_now: WallClockAssertionMsV1,
        lease_id: BoundedId,
        holder_id: BoundedId,
        issued_at: WallClockAssertionMsV1,
        expires_at: WallClockAssertionMsV1,
    },
    MarkStarted {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        started_at: WallClockAssertionMsV1,
    },
    FinishSuccess {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        outcome_ref: BoundedRef,
    },
    FinishFailure {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        outcome_ref: BoundedRef,
    },
    AbortBeforeStart {
        lease_id: BoundedId,
        holder_id: BoundedId,
        lease_epoch: u64,
        reason_ref: BoundedRef,
    },
    RecoverAfterCrash {
        recovery_ref: BoundedRef,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PostCommitEffectV1 {
    None,
    MintActiveLeaseTokenAfterDurableCommit,
    MintStartedAuthorizationAfterDurableCommit,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TransitionPlanV1 {
    expected_store_revision: Option<u64>,
    next_record: VersionedAttemptRecordV1,
    post_commit_effect: PostCommitEffectV1,
}

impl TransitionPlanV1 {
    pub const fn authority_scope(&self) -> LeaseAuthorityScopeV1 {
        LeaseAuthorityScopeV1::PureTransitionPlanningOnly
    }

    pub const fn expected_store_revision(&self) -> Option<u64> {
        self.expected_store_revision
    }

    pub fn next_record(&self) -> &VersionedAttemptRecordV1 {
        &self.next_record
    }

    pub const fn post_commit_effect(&self) -> PostCommitEffectV1 {
        self.post_commit_effect
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LeaseDecisionV1 {
    Apply(TransitionPlanV1),
    NoOp,
    Reject(LeaseRejectionV1),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseRejectionV1 {
    AuthorityKeyMismatch,
    NotPrepared,
    LeaseNotExpired,
    StartedIsConsumed,
    TerminalState,
    LeaseTokenMismatch,
    TerminalOutcomeConflict,
    InvalidLeaseWindow,
    RevisionOverflow,
    LeaseEpochOverflow,
    LeaseExpired,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LeaseModelError {
    EmptyField { field: &'static str },
    FieldTooLong {
        field: &'static str,
        bytes: usize,
        max: usize,
    },
    NonGraphicAscii { field: &'static str },
}

impl fmt::Display for LeaseModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyField { field } => write!(f, "{field} must not be empty"),
            Self::FieldTooLong { field, bytes, max } => {
                write!(f, "{field} is {bytes} bytes; max is {max}")
            }
            Self::NonGraphicAscii { field } => {
                write!(f, "{field} must contain only graphic ASCII")
            }
        }
    }
}

impl std::error::Error for LeaseModelError {}

pub fn plan_transition_v1(
    current: Option<&VersionedAttemptRecordV1>,
    operation: LeaseOperationV1,
) -> LeaseDecisionV1 {
    match operation {
        LeaseOperationV1::EnsurePrepared { key } => ensure_prepared(current, key),
        LeaseOperationV1::AcquireLease {
            lease_id,
            holder_id,
            issued_at,
            expires_at,
        } => acquire_lease(current, lease_id, holder_id, issued_at, expires_at),
        LeaseOperationV1::ReacquireExpiredLease {
            observed_now,
            lease_id,
            holder_id,
            issued_at,
            expires_at,
        } => reacquire_expired(
            current,
            observed_now,
            lease_id,
            holder_id,
            issued_at,
            expires_at,
        ),
        LeaseOperationV1::MarkStarted {
            lease_id,
            holder_id,
            lease_epoch,
            started_at,
        } => mark_started(current, lease_id, holder_id, lease_epoch, started_at),
        LeaseOperationV1::FinishSuccess {
            lease_id,
            holder_id,
            lease_epoch,
            outcome_ref,
        } => finish(
            current,
            true,
            lease_id,
            holder_id,
            lease_epoch,
            outcome_ref,
        ),
        LeaseOperationV1::FinishFailure {
            lease_id,
            holder_id,
            lease_epoch,
            outcome_ref,
        } => finish(
            current,
            false,
            lease_id,
            holder_id,
            lease_epoch,
            outcome_ref,
        ),
        LeaseOperationV1::AbortBeforeStart {
            lease_id,
            holder_id,
            lease_epoch,
            reason_ref,
        } => abort_before_start(
            current,
            lease_id,
            holder_id,
            lease_epoch,
            reason_ref,
        ),
        LeaseOperationV1::RecoverAfterCrash { recovery_ref } => {
            recover_after_crash(current, recovery_ref)
        }
    }
}

fn ensure_prepared(
    current: Option<&VersionedAttemptRecordV1>,
    key: AttemptAuthorityKeyV1,
) -> LeaseDecisionV1 {
    match current {
        None => LeaseDecisionV1::Apply(TransitionPlanV1 {
            expected_store_revision: None,
            next_record: VersionedAttemptRecordV1::prepared(key),
            post_commit_effect: PostCommitEffectV1::None,
        }),
        Some(record) if record.key() == &key => LeaseDecisionV1::NoOp,
        Some(_) => LeaseDecisionV1::Reject(LeaseRejectionV1::AuthorityKeyMismatch),
    }
}

fn acquire_lease(
    current: Option<&VersionedAttemptRecordV1>,
    lease_id: BoundedId,
    holder_id: BoundedId,
    issued_at: WallClockAssertionMsV1,
    expires_at: WallClockAssertionMsV1,
) -> LeaseDecisionV1 {
    if expires_at <= issued_at {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::InvalidLeaseWindow);
    }
    let Some(record) = current else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::NotPrepared);
    };
    let LeaseStateV1::Prepared { lease_epoch } = record.state() else {
        return LeaseDecisionV1::Reject(if record.state().is_terminal() {
            LeaseRejectionV1::TerminalState
        } else if matches!(record.state(), LeaseStateV1::Started { .. }) {
            LeaseRejectionV1::StartedIsConsumed
        } else {
            LeaseRejectionV1::NotPrepared
        });
    };
    let Some(next_epoch) = lease_epoch.checked_add(1) else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseEpochOverflow);
    };
    plan_from_existing(
        record,
        LeaseStateV1::Leased {
            lease_id,
            holder_id,
            lease_epoch: next_epoch,
            issued_at,
            expires_at,
        },
        PostCommitEffectV1::MintActiveLeaseTokenAfterDurableCommit,
    )
}

fn reacquire_expired(
    current: Option<&VersionedAttemptRecordV1>,
    observed_now: WallClockAssertionMsV1,
    lease_id: BoundedId,
    holder_id: BoundedId,
    issued_at: WallClockAssertionMsV1,
    expires_at: WallClockAssertionMsV1,
) -> LeaseDecisionV1 {
    if expires_at <= issued_at {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::InvalidLeaseWindow);
    }
    let Some(record) = current else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::NotPrepared);
    };
    let LeaseStateV1::Leased {
        lease_epoch,
        expires_at: current_expiry,
        ..
    } = record.state()
    else {
        return LeaseDecisionV1::Reject(if matches!(record.state(), LeaseStateV1::Started { .. }) {
            LeaseRejectionV1::StartedIsConsumed
        } else if record.state().is_terminal() {
            LeaseRejectionV1::TerminalState
        } else {
            LeaseRejectionV1::NotPrepared
        });
    };
    if observed_now < *current_expiry {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseNotExpired);
    }
    if issued_at > observed_now || expires_at <= observed_now {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::InvalidLeaseWindow);
    }
    let Some(next_epoch) = lease_epoch.checked_add(1) else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseEpochOverflow);
    };
    plan_from_existing(
        record,
        LeaseStateV1::Leased {
            lease_id,
            holder_id,
            lease_epoch: next_epoch,
            issued_at,
            expires_at,
        },
        PostCommitEffectV1::MintActiveLeaseTokenAfterDurableCommit,
    )
}

fn mark_started(
    current: Option<&VersionedAttemptRecordV1>,
    lease_id: BoundedId,
    holder_id: BoundedId,
    lease_epoch: u64,
    started_at: WallClockAssertionMsV1,
) -> LeaseDecisionV1 {
    let Some(record) = current else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::NotPrepared);
    };
    let LeaseStateV1::Leased {
        lease_id: current_lease,
        holder_id: current_holder,
        lease_epoch: current_epoch,
        expires_at,
        ..
    } = record.state()
    else {
        return LeaseDecisionV1::Reject(if matches!(record.state(), LeaseStateV1::Started { .. }) {
            LeaseRejectionV1::StartedIsConsumed
        } else if record.state().is_terminal() {
            LeaseRejectionV1::TerminalState
        } else {
            LeaseRejectionV1::NotPrepared
        });
    };
    if current_lease != &lease_id || current_holder != &holder_id || *current_epoch != lease_epoch {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseTokenMismatch);
    }
    if started_at >= *expires_at {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseExpired);
    }
    plan_from_existing(
        record,
        LeaseStateV1::Started {
            lease_id,
            holder_id,
            lease_epoch,
            started_at,
        },
        PostCommitEffectV1::MintStartedAuthorizationAfterDurableCommit,
    )
}

fn finish(
    current: Option<&VersionedAttemptRecordV1>,
    success: bool,
    lease_id: BoundedId,
    holder_id: BoundedId,
    lease_epoch: u64,
    outcome_ref: BoundedRef,
) -> LeaseDecisionV1 {
    let Some(record) = current else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::NotPrepared);
    };

    match record.state() {
        LeaseStateV1::Started {
            lease_id: current_lease,
            holder_id: current_holder,
            lease_epoch: current_epoch,
            ..
        } => {
            if current_lease != &lease_id
                || current_holder != &holder_id
                || *current_epoch != lease_epoch
            {
                return LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseTokenMismatch);
            }
            let next = if success {
                LeaseStateV1::FinishedSuccess {
                    lease_id,
                    holder_id,
                    lease_epoch,
                    outcome_ref,
                }
            } else {
                LeaseStateV1::FinishedFailure {
                    lease_id,
                    holder_id,
                    lease_epoch,
                    outcome_ref,
                }
            };
            plan_from_existing(record, next, PostCommitEffectV1::None)
        }
        LeaseStateV1::FinishedSuccess {
            lease_id: old_lease,
            holder_id: old_holder,
            lease_epoch: old_epoch,
            outcome_ref: old_outcome,
        } if success
            && old_lease == &lease_id
            && old_holder == &holder_id
            && *old_epoch == lease_epoch
            && old_outcome == &outcome_ref =>
        {
            LeaseDecisionV1::NoOp
        }
        LeaseStateV1::FinishedFailure {
            lease_id: old_lease,
            holder_id: old_holder,
            lease_epoch: old_epoch,
            outcome_ref: old_outcome,
        } if !success
            && old_lease == &lease_id
            && old_holder == &holder_id
            && *old_epoch == lease_epoch
            && old_outcome == &outcome_ref =>
        {
            LeaseDecisionV1::NoOp
        }
        state if state.is_terminal() => {
            LeaseDecisionV1::Reject(LeaseRejectionV1::TerminalOutcomeConflict)
        }
        _ => LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseTokenMismatch),
    }
}

fn abort_before_start(
    current: Option<&VersionedAttemptRecordV1>,
    lease_id: BoundedId,
    holder_id: BoundedId,
    lease_epoch: u64,
    reason_ref: BoundedRef,
) -> LeaseDecisionV1 {
    let Some(record) = current else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::NotPrepared);
    };
    let LeaseStateV1::Leased {
        lease_id: current_lease,
        holder_id: current_holder,
        lease_epoch: current_epoch,
        ..
    } = record.state()
    else {
        return LeaseDecisionV1::Reject(if record.state().is_terminal() {
            LeaseRejectionV1::TerminalState
        } else {
            LeaseRejectionV1::LeaseTokenMismatch
        });
    };
    if current_lease != &lease_id || current_holder != &holder_id || *current_epoch != lease_epoch {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseTokenMismatch);
    }
    plan_from_existing(
        record,
        LeaseStateV1::AbortedBeforeStart {
            lease_id,
            holder_id,
            lease_epoch,
            reason_ref,
        },
        PostCommitEffectV1::None,
    )
}

fn recover_after_crash(
    current: Option<&VersionedAttemptRecordV1>,
    recovery_ref: BoundedRef,
) -> LeaseDecisionV1 {
    let Some(record) = current else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::NotPrepared);
    };
    let LeaseStateV1::Started {
        lease_id,
        holder_id,
        lease_epoch,
        ..
    } = record.state()
    else {
        return LeaseDecisionV1::Reject(if record.state().is_terminal() {
            LeaseRejectionV1::TerminalState
        } else {
            LeaseRejectionV1::NotPrepared
        });
    };
    plan_from_existing(
        record,
        LeaseStateV1::IndeterminateAfterCrash {
            lease_id: lease_id.clone(),
            holder_id: holder_id.clone(),
            lease_epoch: *lease_epoch,
            recovery_ref,
        },
        PostCommitEffectV1::None,
    )
}

fn plan_from_existing(
    current: &VersionedAttemptRecordV1,
    next_state: LeaseStateV1,
    post_commit_effect: PostCommitEffectV1,
) -> LeaseDecisionV1 {
    let Some(next_revision) = current.store_revision().checked_add(1) else {
        return LeaseDecisionV1::Reject(LeaseRejectionV1::RevisionOverflow);
    };
    LeaseDecisionV1::Apply(TransitionPlanV1 {
        expected_store_revision: Some(current.store_revision()),
        next_record: VersionedAttemptRecordV1::from_parts(
            current.key().clone(),
            next_revision,
            next_state,
        ),
        post_commit_effect,
    })
}

fn validate_graphic_ascii(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), LeaseModelError> {
    if value.is_empty() {
        return Err(LeaseModelError::EmptyField { field });
    }
    if value.len() > max {
        return Err(LeaseModelError::FieldTooLong {
            field,
            bytes: value.len(),
            max,
        });
    }
    if !value.bytes().all(|b| b.is_ascii_graphic()) {
        return Err(LeaseModelError::NonGraphicAscii { field });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> BoundedId {
        BoundedId::new(value).unwrap()
    }

    fn reference(value: &str) -> BoundedRef {
        BoundedRef::new(value).unwrap()
    }

    fn key(seed: u8, attempt: &str, profile: &str) -> AttemptAuthorityKeyV1 {
        AttemptAuthorityKeyV1::new(
            AdmissionCommitmentV1::from_bytes([seed; 32]),
            id(attempt),
            id(profile),
        )
    }

    fn prepared() -> VersionedAttemptRecordV1 {
        VersionedAttemptRecordV1::prepared(key(1, "attempt-A", "exec-v1"))
    }

    fn applied(decision: LeaseDecisionV1) -> TransitionPlanV1 {
        match decision {
            LeaseDecisionV1::Apply(plan) => plan,
            other => panic!("expected apply, got {other:?}"),
        }
    }

    fn leased() -> VersionedAttemptRecordV1 {
        let p = prepared();
        applied(plan_transition_v1(
            Some(&p),
            LeaseOperationV1::AcquireLease {
                lease_id: id("L1"),
                holder_id: id("H1"),
                issued_at: WallClockAssertionMsV1(1_000),
                expires_at: WallClockAssertionMsV1(2_000),
            },
        ))
        .next_record
    }

    fn started() -> VersionedAttemptRecordV1 {
        let l = leased();
        applied(plan_transition_v1(
            Some(&l),
            LeaseOperationV1::MarkStarted {
                lease_id: id("L1"),
                holder_id: id("H1"),
                lease_epoch: 1,
                started_at: WallClockAssertionMsV1(1_100),
            },
        ))
        .next_record
    }

    #[test]
    fn ensure_prepared_is_idempotent_not_reset() {
        let k = key(1, "attempt-A", "exec-v1");
        let first = applied(plan_transition_v1(
            None,
            LeaseOperationV1::EnsurePrepared { key: k.clone() },
        ));
        assert_eq!(first.expected_store_revision(), None);
        assert_eq!(
            plan_transition_v1(
                Some(first.next_record()),
                LeaseOperationV1::EnsurePrepared { key: k }
            ),
            LeaseDecisionV1::NoOp
        );
    }

    #[test]
    fn lease_plan_requires_post_commit_token_minting() {
        let p = prepared();
        let plan = applied(plan_transition_v1(
            Some(&p),
            LeaseOperationV1::AcquireLease {
                lease_id: id("L1"),
                holder_id: id("H1"),
                issued_at: WallClockAssertionMsV1(1_000),
                expires_at: WallClockAssertionMsV1(2_000),
            },
        ));
        assert_eq!(plan.expected_store_revision(), Some(0));
        assert_eq!(
            plan.post_commit_effect(),
            PostCommitEffectV1::MintActiveLeaseTokenAfterDurableCommit
        );
        assert_eq!(
            plan.authority_scope(),
            LeaseAuthorityScopeV1::PureTransitionPlanningOnly
        );
    }

    #[test]
    fn stale_start_tokens_are_rejected() {
        let l = leased();
        for (lease, holder, epoch) in [("BAD", "H1", 1), ("L1", "BAD", 1), ("L1", "H1", 0)] {
            assert_eq!(
                plan_transition_v1(
                    Some(&l),
                    LeaseOperationV1::MarkStarted {
                        lease_id: id(lease),
                        holder_id: id(holder),
                        lease_epoch: epoch,
                        started_at: WallClockAssertionMsV1(1_100),
                    }
                ),
                LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseTokenMismatch)
            );
        }
    }

    #[test]
    fn expired_lease_cannot_start_even_before_reacquire() {
        let l = leased();
        assert_eq!(
            plan_transition_v1(
                Some(&l),
                LeaseOperationV1::MarkStarted {
                    lease_id: id("L1"),
                    holder_id: id("H1"),
                    lease_epoch: 1,
                    started_at: WallClockAssertionMsV1(2_000),
                }
            ),
            LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseExpired)
        );
    }

    #[test]
    fn forward_clock_recovery_invalidates_old_epoch() {
        let l1 = leased();
        let l2 = applied(plan_transition_v1(
            Some(&l1),
            LeaseOperationV1::ReacquireExpiredLease {
                observed_now: WallClockAssertionMsV1(9_000_000),
                lease_id: id("L2"),
                holder_id: id("H2"),
                issued_at: WallClockAssertionMsV1(9_000_000),
                expires_at: WallClockAssertionMsV1(9_001_000),
            },
        ))
        .next_record;

        assert_eq!(l2.state().lease_epoch(), 2);
        assert_eq!(
            plan_transition_v1(
                Some(&l2),
                LeaseOperationV1::MarkStarted {
                    lease_id: id("L1"),
                    holder_id: id("H1"),
                    lease_epoch: 1,
                    started_at: WallClockAssertionMsV1(9_000_100),
                }
            ),
            LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseTokenMismatch)
        );
    }

    #[test]
    fn rollback_clock_delays_recovery() {
        let l = leased();
        assert_eq!(
            plan_transition_v1(
                Some(&l),
                LeaseOperationV1::ReacquireExpiredLease {
                    observed_now: WallClockAssertionMsV1(1_500),
                    lease_id: id("L2"),
                    holder_id: id("H2"),
                    issued_at: WallClockAssertionMsV1(1_500),
                    expires_at: WallClockAssertionMsV1(2_500),
                }
            ),
            LeaseDecisionV1::Reject(LeaseRejectionV1::LeaseNotExpired)
        );
    }

    #[test]
    fn started_never_releases_on_wall_clock_expiry() {
        let s = started();
        assert_eq!(
            plan_transition_v1(
                Some(&s),
                LeaseOperationV1::ReacquireExpiredLease {
                    observed_now: WallClockAssertionMsV1(u64::MAX),
                    lease_id: id("L2"),
                    holder_id: id("H2"),
                    issued_at: WallClockAssertionMsV1(u64::MAX - 1),
                    expires_at: WallClockAssertionMsV1(u64::MAX),
                }
            ),
            LeaseDecisionV1::Reject(LeaseRejectionV1::StartedIsConsumed)
        );
    }

    #[test]
    fn failed_attempt_is_terminal_and_same_terminal_write_is_idempotent() {
        let s = started();
        let terminal = applied(plan_transition_v1(
            Some(&s),
            LeaseOperationV1::FinishFailure {
                lease_id: id("L1"),
                holder_id: id("H1"),
                lease_epoch: 1,
                outcome_ref: reference("tcp-observation:failure:A"),
            },
        ))
        .next_record;

        assert_eq!(
            plan_transition_v1(
                Some(&terminal),
                LeaseOperationV1::FinishFailure {
                    lease_id: id("L1"),
                    holder_id: id("H1"),
                    lease_epoch: 1,
                    outcome_ref: reference("tcp-observation:failure:A"),
                }
            ),
            LeaseDecisionV1::NoOp
        );

        assert_eq!(
            plan_transition_v1(
                Some(&terminal),
                LeaseOperationV1::FinishSuccess {
                    lease_id: id("L1"),
                    holder_id: id("H1"),
                    lease_epoch: 1,
                    outcome_ref: reference("tcp-observation:success:B"),
                }
            ),
            LeaseDecisionV1::Reject(LeaseRejectionV1::TerminalOutcomeConflict)
        );
    }

    #[test]
    fn started_crash_is_terminal_indeterminate() {
        let s = started();
        let terminal = applied(plan_transition_v1(
            Some(&s),
            LeaseOperationV1::RecoverAfterCrash {
                recovery_ref: reference("worker-recovery:A"),
            },
        ))
        .next_record;
        assert!(matches!(
            terminal.state(),
            LeaseStateV1::IndeterminateAfterCrash { .. }
        ));
        assert_eq!(
            plan_transition_v1(
                Some(&terminal),
                LeaseOperationV1::ReacquireExpiredLease {
                    observed_now: WallClockAssertionMsV1(u64::MAX),
                    lease_id: id("L2"),
                    holder_id: id("H2"),
                    issued_at: WallClockAssertionMsV1(u64::MAX - 1),
                    expires_at: WallClockAssertionMsV1(u64::MAX),
                }
            ),
            LeaseDecisionV1::Reject(LeaseRejectionV1::TerminalState)
        );
    }

    #[test]
    fn authority_key_is_full_tuple() {
        let a = key(1, "attempt-X", "P1");
        let b = key(2, "attempt-X", "P1");
        let c = key(1, "attempt-Y", "P1");
        let d = key(1, "attempt-X", "P2");
        assert_ne!(a, b);
        assert_ne!(a, c);
        assert_ne!(a, d);
    }

    #[test]
    fn debug_redacts_authority_material() {
        let secret = AttemptAuthorityKeyV1::new(
            AdmissionCommitmentV1::from_bytes([7; 32]),
            id("secret-attempt"),
            id("secret-profile"),
        );
        let rendered = format!("{secret:?}");
        assert!(!rendered.contains("secret-attempt"));
        assert!(!rendered.contains("secret-profile"));
    }
}

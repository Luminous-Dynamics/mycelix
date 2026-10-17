//! Short-lived, single-use guard for a protected side-effect boundary.
//!
//! This is a reference model, not a production durable/replicated lease service.
//! It re-evaluates the pure policy kernel immediately before the callback, binds
//! the exact effect target, and marks a lease consumed before invoking the
//! callback. A production adapter must atomically persist consumption with the
//! protected operation (or use a qualified fencing protocol) and refresh trusted
//! profile, agreement, classification, consent, and revocation evidence.

use std::collections::HashSet;

use super::{
    evaluate_transfer, BoundaryAction, DataClass, PolicyDecision, TransferRequest, VerifiedProfile,
};

const MAX_LEASE_TTL_MS: u64 = 5_000;
const MAX_REQUEST_ID_BYTES: usize = 256;
const MAX_RESOURCE_ID_BYTES: usize = 512;
const MAX_RECIPIENT_ID_BYTES: usize = 512;
const MAX_PURPOSE_BYTES: usize = 512;
const LEASE_ID_HEX_BYTES: usize = 64;

/// Stable, non-sensitive lease rejection category. Never place target fields in
/// this error: targets may contain private resource, recipient, or purpose IDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeaseError {
    PolicyDenied,
    DecisionNotFresh,
    InvalidTarget,
    InvalidLeaseId,
    InvalidTtl,
    InvalidDecisionBinding,
    ClockOverflow,
    ClockBeforeIssue,
    Expired,
    TargetMismatch,
    PolicyChanged,
    Replay,
}

/// Exact effect context. Treat every field as sensitive operational metadata:
/// do not log or serialize this object into public audit streams.
#[derive(Clone, PartialEq, Eq)]
pub struct EffectTarget {
    pub request_id: String,
    pub action: BoundaryAction,
    pub resource_id: String,
    pub content_digest: String,
    pub recipient_id: Option<String>,
    pub purpose: Option<String>,
}

impl EffectTarget {
    /// Construct the request-side target. The final sink should independently
    /// construct its observed target from the operation it is about to perform.
    pub fn from_request(request_id: &str, request: &TransferRequest<'_>) -> Self {
        Self {
            request_id: request_id.to_owned(),
            action: request.action,
            resource_id: request.resource_id.to_owned(),
            content_digest: request.content_digest.to_owned(),
            recipient_id: request.recipient_id.map(str::to_owned),
            purpose: request.purpose.map(str::to_owned),
        }
    }

    fn validate(&self) -> Result<(), LeaseError> {
        if !valid_text(&self.request_id, MAX_REQUEST_ID_BYTES)
            || !valid_text(&self.resource_id, MAX_RESOURCE_ID_BYTES)
            || !valid_digest(&self.content_digest)
            || !valid_optional_text(&self.recipient_id, MAX_RECIPIENT_ID_BYTES)
            || !valid_optional_text(&self.purpose, MAX_PURPOSE_BYTES)
        {
            return Err(LeaseError::InvalidTarget);
        }
        Ok(())
    }
}

#[derive(Clone, PartialEq, Eq)]
struct PolicyBinding {
    agreement_id: String,
    agreement_version: String,
    agreement_digest: String,
    source_profile_id: String,
    source_profile_version: String,
    source_profile_digest: String,
    destination_profile_id: String,
    destination_profile_version: String,
    destination_profile_digest: String,
    action: BoundaryAction,
    data_class: DataClass,
    content_digest: String,
}

impl PolicyBinding {
    fn from_decision(decision: &PolicyDecision) -> Self {
        Self {
            agreement_id: decision.agreement_id.clone(),
            agreement_version: decision.agreement_version.clone(),
            agreement_digest: decision.agreement_digest.clone(),
            source_profile_id: decision.source_profile_id.clone(),
            source_profile_version: decision.source_profile_version.clone(),
            source_profile_digest: decision.source_profile_digest.clone(),
            destination_profile_id: decision.destination_profile_id.clone(),
            destination_profile_version: decision.destination_profile_version.clone(),
            destination_profile_digest: decision.destination_profile_digest.clone(),
            action: decision.action,
            data_class: decision.data_class,
            content_digest: decision.content_digest.clone(),
        }
    }

    fn validate(&self) -> bool {
        valid_text(&self.agreement_id, MAX_RESOURCE_ID_BYTES)
            && valid_text(&self.agreement_version, MAX_RESOURCE_ID_BYTES)
            && valid_digest(&self.agreement_digest)
            && valid_text(&self.source_profile_id, MAX_RESOURCE_ID_BYTES)
            && valid_text(&self.source_profile_version, MAX_RESOURCE_ID_BYTES)
            && valid_digest(&self.source_profile_digest)
            && valid_text(&self.destination_profile_id, MAX_RESOURCE_ID_BYTES)
            && valid_text(&self.destination_profile_version, MAX_RESOURCE_ID_BYTES)
            && valid_digest(&self.destination_profile_digest)
            && valid_digest(&self.content_digest)
    }
}

/// Opaque in-process lease. There is intentionally no Clone, Debug, or wire
/// serialization API. A deployment that needs cross-process leases must define
/// authenticated encoding, key rotation, expiry, fencing, and durable replay
/// handling separately.
pub struct AuthorizationLease {
    lease_id: String,
    target: EffectTarget,
    policy: PolicyBinding,
    issued_at_unix_ms: u64,
    expires_at_unix_ms: u64,
}

impl AuthorizationLease {
    /// Issue only from a current, allowed decision over this exact request.
    /// lease_id must be 32 bytes of cryptographically random entropy encoded as
    /// 64 lowercase hexadecimal characters by the trusted adapter.
    pub fn issue(
        source: &VerifiedProfile,
        destination: &VerifiedProfile,
        request: &TransferRequest<'_>,
        request_id: &str,
        lease_id: &str,
        now_ms: u64,
        ttl_ms: u64,
    ) -> Result<Self, LeaseError> {
        if request.evaluated_at_unix_ms != now_ms {
            return Err(LeaseError::DecisionNotFresh);
        }
        if ttl_ms == 0 || ttl_ms > MAX_LEASE_TTL_MS {
            return Err(LeaseError::InvalidTtl);
        }
        if !valid_nonce(lease_id) {
            return Err(LeaseError::InvalidLeaseId);
        }

        let target = EffectTarget::from_request(request_id, request);
        target.validate()?;

        let decision = evaluate_transfer(source, destination, request);
        if !decision.allowed || !decision.reasons.is_empty() {
            return Err(LeaseError::PolicyDenied);
        }
        let policy = PolicyBinding::from_decision(&decision);
        if !policy.validate() {
            return Err(LeaseError::InvalidDecisionBinding);
        }
        let expires_at_unix_ms = now_ms
            .checked_add(ttl_ms)
            .ok_or(LeaseError::ClockOverflow)?;

        Ok(Self {
            lease_id: lease_id.to_owned(),
            target,
            policy,
            issued_at_unix_ms: now_ms,
            expires_at_unix_ms,
        })
    }

    /// Stable nonce handle for a trusted local ledger. It is not sufficient
    /// authorization by itself and must not be used as a bearer credential.
    pub fn lease_id(&self) -> &str {
        &self.lease_id
    }

    pub fn expires_at_unix_ms(&self) -> u64 {
        self.expires_at_unix_ms
    }
}

/// Result of consuming a lease. If the effect callback errors, the lease is
/// still consumed: its outcome may be uncertain, so blind replay is forbidden.
#[derive(Debug, PartialEq, Eq)]
pub enum ConsumeError<E> {
    Rejected(LeaseError),
    EffectFailed(E),
}

/// Process-local reference ledger. It is neither crash durable nor shared
/// across nodes. Production must replace this with an atomic durable ledger or
/// a storage-native fencing protocol.
#[derive(Default)]
pub struct LeaseLedger {
    consumed_lease_ids: HashSet<String>,
}

impl LeaseLedger {
    /// Re-evaluate the policy with a trusted, current timestamp, compare the
    /// observed sink target to the issued target, consume the nonce, then run the
    /// side effect. The ledger's mutable borrow serializes users of this
    /// reference instance. Atomicity across process crashes or other nodes is
    /// explicitly outside this type's guarantee.
    pub fn consume_before_effect<E, F>(
        &mut self,
        lease: AuthorizationLease,
        source: &VerifiedProfile,
        destination: &VerifiedProfile,
        request: &TransferRequest<'_>,
        observed_target: &EffectTarget,
        now_ms: u64,
        effect: F,
    ) -> Result<(), ConsumeError<E>>
    where
        F: FnOnce() -> Result<(), E>,
    {
        let reject = |error| ConsumeError::Rejected(error);

        if now_ms < lease.issued_at_unix_ms {
            return Err(reject(LeaseError::ClockBeforeIssue));
        }
        if now_ms >= lease.expires_at_unix_ms {
            return Err(reject(LeaseError::Expired));
        }
        if request.evaluated_at_unix_ms != now_ms {
            return Err(reject(LeaseError::DecisionNotFresh));
        }

        let request_target = EffectTarget::from_request(&observed_target.request_id, request);
        if observed_target != &lease.target || observed_target != &request_target {
            return Err(reject(LeaseError::TargetMismatch));
        }
        observed_target.validate().map_err(reject)?;

        let current_decision = evaluate_transfer(source, destination, request);
        if !current_decision.allowed || !current_decision.reasons.is_empty() {
            return Err(reject(LeaseError::PolicyDenied));
        }
        if PolicyBinding::from_decision(&current_decision) != lease.policy {
            return Err(reject(LeaseError::PolicyChanged));
        }

        if !self.consumed_lease_ids.insert(lease.lease_id.clone()) {
            return Err(reject(LeaseError::Replay));
        }

        effect().map_err(ConsumeError::EffectFailed)
    }
}

fn valid_nonce(value: &str) -> bool {
    value.len() == LEASE_ID_HEX_BYTES
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_optional_text(value: &Option<String>, max_bytes: usize) -> bool {
    value
        .as_ref()
        .is_none_or(|inner| valid_text(inner, max_bytes))
}

fn valid_text(value: &str, max_bytes: usize) -> bool {
    !value.trim().is_empty()
        && value.len() <= max_bytes
        && !value.chars().any(char::is_control)
}

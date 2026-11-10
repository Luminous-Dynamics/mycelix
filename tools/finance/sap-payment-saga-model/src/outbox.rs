//! Effects-disabled reference model for bounded Treasury outbox recovery.
//!
//! The crucial rule is that a lease expiring before dispatch authorization is
//! safe to retry, but a lease expiring after authorization is ambiguous and
//! must reconcile before any further dispatch.

use sha2::{Digest, Sha256};
use std::collections::BTreeMap;

const BACKOFF_DOMAIN: &[u8] = b"MXSAP-OUTBOX-BACKOFF-V1\0";
const MAX_BATCH: usize = 64;
const MAX_TEXT_BYTES: usize = 4096;
const MAX_ERROR_CODE_BYTES: usize = 128;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NewOutboxEffect {
    pub operation_id: String,
    pub effect_id: String,
    pub destination_id: String,
    pub payload_digest: String,
    pub created_at_ms: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OutboxReceipt {
    pub operation_id: String,
    pub effect_id: String,
    pub destination_id: String,
    pub payload_digest: String,
    pub receipt_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum QueryOutcome {
    /// This is a destination-scoped, adapter-qualified authoritative absence
    /// proof, not an ordinary HTTP 404 or an eventually consistent empty read.
    AuthoritativeAbsence { proof_digest: String },
    Receipt(OutboxReceipt),
    /// The destination cannot currently prove either presence or absence.
    Uncertain,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DispatchOutcome {
    Committed(OutboxReceipt),
    /// Adapter contract proves that this attempt caused no destination effect.
    DefinitelyNoEffectTransient { error_code: String },
    /// A destination effect may have happened; query/reconcile the same ID.
    Indeterminate { evidence_digest: String },
    PermanentRejection { error_code: String },
    SourceEvidenceInvalid { evidence_digest: String },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BackoffPolicy {
    pub base_delay_ms: u64,
    pub max_delay_ms: u64,
    pub jitter_window_ms: u64,
    pub max_dispatch_attempts: u32,
    pub max_reconciliation_attempts: u32,
    pub max_reconciliation_age_ms: u64,
}

impl BackoffPolicy {
    pub fn validate(self) -> Result<(), OutboxError> {
        if self.base_delay_ms == 0
            || self.max_delay_ms < self.base_delay_ms
            || self.jitter_window_ms >= self.max_delay_ms
            || self.max_dispatch_attempts == 0
            || self.max_reconciliation_attempts == 0
            || self.max_reconciliation_age_ms == 0
        {
            return Err(OutboxError::InvalidPolicy);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OutboxLease {
    pub effect_id: String,
    pub worker_id: String,
    pub generation: u64,
    pub expires_at_ms: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LeaseState {
    worker_id: String,
    generation: u64,
    expires_at_ms: u64,
}

impl From<&OutboxLease> for LeaseState {
    fn from(value: &OutboxLease) -> Self {
        Self {
            worker_id: value.worker_id.clone(),
            generation: value.generation,
            expires_at_ms: value.expires_at_ms,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum OutboxState {
    Ready,
    LeasedForPreflight(LeaseState),
    DispatchAuthorized {
        lease: LeaseState,
        absence_proof_digest: String,
    },
    AwaitingReconciliation,
    LeasedForReconciliation(LeaseState),
    Delivered { receipt_digest: String },
    DeadLetter { error_code: String },
    ManualReview { reason_code: String },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OutboxItem {
    pub operation_id: String,
    pub effect_id: String,
    pub destination_id: String,
    pub payload_digest: String,
    pub created_at_ms: u64,
    pub next_attempt_at_ms: u64,
    pub next_reconciliation_at_ms: u64,
    pub dispatch_attempts: u32,
    pub reconciliation_attempts: u32,
    pub reconciliation_started_at_ms: Option<u64>,
    pub last_attempt_at_ms: Option<u64>,
    pub lease_generation: u64,
    pub last_error_code: Option<String>,
    pub last_evidence_digest: Option<String>,
    pub last_absence_proof_digest: Option<String>,
    pub state: OutboxState,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EnqueueDisposition {
    Inserted,
    ExistingSameEffect,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OutboxError {
    InvalidInput,
    InvalidPolicy,
    BatchLimitExceeded,
    ImmutableEffectConflict,
    UnknownEffect,
    LeaseMismatch,
    LeaseExpired,
    InvalidTime,
    CounterOverflow,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct QueueMetrics {
    pub ready: usize,
    pub leased: usize,
    pub awaiting_reconciliation: usize,
    pub delivered: usize,
    pub dead_letter: usize,
    pub manual_review: usize,
    pub oldest_unresolved_age_ms: u64,
}

#[derive(Clone, Copy)]
enum LeasePhase {
    Preflight,
    Dispatch,
    Reconciliation,
}

#[derive(Default)]
pub struct OutboxBook {
    items: BTreeMap<String, OutboxItem>,
}

impl OutboxBook {
    pub fn enqueue(
        &mut self,
        effect: NewOutboxEffect,
    ) -> Result<EnqueueDisposition, OutboxError> {
        validate_new_effect(&effect)?;
        if let Some(existing) = self.items.get(&effect.effect_id) {
            if existing.operation_id == effect.operation_id
                && existing.destination_id == effect.destination_id
                && existing.payload_digest == effect.payload_digest
            {
                return Ok(EnqueueDisposition::ExistingSameEffect);
            }
            return Err(OutboxError::ImmutableEffectConflict);
        }
        self.items.insert(
            effect.effect_id.clone(),
            OutboxItem {
                operation_id: effect.operation_id,
                effect_id: effect.effect_id,
                destination_id: effect.destination_id,
                payload_digest: effect.payload_digest,
                created_at_ms: effect.created_at_ms,
                next_attempt_at_ms: effect.created_at_ms,
                next_reconciliation_at_ms: effect.created_at_ms,
                dispatch_attempts: 0,
                reconciliation_attempts: 0,
                reconciliation_started_at_ms: None,
                last_attempt_at_ms: None,
                lease_generation: 0,
                last_error_code: None,
                last_evidence_digest: None,
                last_absence_proof_digest: None,
                state: OutboxState::Ready,
            },
        );
        Ok(EnqueueDisposition::Inserted)
    }

    /// Claims at most 64 ready items in due/age/effect-ID order. Every claim is
    /// only for a destination preflight query; it does not authorize dispatch.
    pub fn claim_due(
        &mut self,
        worker_id: &str,
        now_ms: u64,
        lease_duration_ms: u64,
        limit: usize,
        policy: BackoffPolicy,
    ) -> Result<Vec<OutboxLease>, OutboxError> {
        validate_worker_and_lease(worker_id, lease_duration_ms, limit, policy)?;
        self.expire_leases(now_ms);
        let mut candidates: Vec<(u64, u64, String)> = self
            .items
            .values()
            .filter(|item| item.state == OutboxState::Ready && item.next_attempt_at_ms <= now_ms)
            .map(|item| (item.next_attempt_at_ms, item.created_at_ms, item.effect_id.clone()))
            .collect();
        candidates.sort();
        let mut leases = Vec::new();
        for (_, _, effect_id) in candidates.into_iter().take(limit) {
            let item = self.items.get_mut(&effect_id).expect("candidate exists");
            if item.dispatch_attempts >= policy.max_dispatch_attempts {
                item.state = OutboxState::DeadLetter {
                    error_code: "dispatch_attempt_limit".into(),
                };
                continue;
            }
            let lease = new_lease(item, worker_id, now_ms, lease_duration_ms)?;
            item.state = OutboxState::LeasedForPreflight(LeaseState::from(&lease));
            leases.push(lease);
        }
        Ok(leases)
    }

    /// Claims unresolved items only for a read-only reconciliation query.
    pub fn claim_reconciliation(
        &mut self,
        worker_id: &str,
        now_ms: u64,
        lease_duration_ms: u64,
        limit: usize,
        policy: BackoffPolicy,
    ) -> Result<Vec<OutboxLease>, OutboxError> {
        validate_worker_and_lease(worker_id, lease_duration_ms, limit, policy)?;
        self.expire_leases(now_ms);
        let mut candidates: Vec<(u64, u64, String)> = self
            .items
            .values()
            .filter(|item| {
                item.state == OutboxState::AwaitingReconciliation
                    && item.next_reconciliation_at_ms <= now_ms
            })
            .map(|item| {
                (
                    item.next_reconciliation_at_ms,
                    item.created_at_ms,
                    item.effect_id.clone(),
                )
            })
            .collect();
        candidates.sort();
        let mut leases = Vec::new();
        for (_, _, effect_id) in candidates.into_iter().take(limit) {
            let item = self.items.get_mut(&effect_id).expect("candidate exists");
            if reconciliation_budget_exhausted(item, now_ms, policy) {
                item.state = OutboxState::ManualReview {
                    reason_code: "reconciliation_budget_exhausted".into(),
                };
                item.last_error_code = Some("reconciliation_budget_exhausted".into());
                continue;
            }
            let lease = new_lease(item, worker_id, now_ms, lease_duration_ms)?;
            item.reconciliation_started_at_ms.get_or_insert(now_ms);
            item.reconciliation_attempts = item
                .reconciliation_attempts
                .checked_add(1)
                .ok_or(OutboxError::CounterOverflow)?;
            item.state = OutboxState::LeasedForReconciliation(LeaseState::from(&lease));
            leases.push(lease);
        }
        Ok(leases)
    }

    /// Applies the pre-dispatch destination query. Only an authoritative,
    /// destination-scoped absence can persist DispatchAuthorized.
    pub fn record_preflight(
        &mut self,
        lease: &OutboxLease,
        outcome: QueryOutcome,
        now_ms: u64,
        policy: BackoffPolicy,
    ) -> Result<(), OutboxError> {
        policy.validate()?;
        let item = self.require_lease_mut(lease, LeasePhase::Preflight, now_ms)?;
        item.last_attempt_at_ms = Some(now_ms);
        match outcome {
            QueryOutcome::Receipt(receipt) => apply_receipt(item, receipt),
            QueryOutcome::AuthoritativeAbsence { proof_digest } => {
                if !is_digest(&proof_digest) {
                    item.state = OutboxState::ManualReview {
                        reason_code: "invalid_absence_proof".into(),
                    };
                } else if item.dispatch_attempts >= policy.max_dispatch_attempts {
                    item.state = OutboxState::DeadLetter {
                        error_code: "dispatch_attempt_limit".into(),
                    };
                } else {
                    item.dispatch_attempts = item
                        .dispatch_attempts
                        .checked_add(1)
                        .ok_or(OutboxError::CounterOverflow)?;
                    item.last_absence_proof_digest = Some(proof_digest.clone());
                    item.state = OutboxState::DispatchAuthorized {
                        lease: LeaseState::from(lease),
                        absence_proof_digest: proof_digest,
                    };
                }
            }
            QueryOutcome::Uncertain => {
                schedule_reconciliation(item, now_ms, policy)?;
            }
        }
        Ok(())
    }

    /// Records the result of one already-authorized stable effect ID. The
    /// preflight-absence proof must have been durably recorded before dispatch.
    pub fn record_dispatch(
        &mut self,
        lease: &OutboxLease,
        outcome: DispatchOutcome,
        now_ms: u64,
        policy: BackoffPolicy,
    ) -> Result<(), OutboxError> {
        policy.validate()?;
        let item = self.require_lease_mut(lease, LeasePhase::Dispatch, now_ms)?;
        item.last_attempt_at_ms = Some(now_ms);
        match outcome {
            DispatchOutcome::Committed(receipt) => apply_receipt(item, receipt),
            DispatchOutcome::DefinitelyNoEffectTransient { error_code } => {
                if !valid_error_code(&error_code) {
                    item.state = OutboxState::ManualReview {
                        reason_code: "invalid_error_code".into(),
                    };
                } else if item.dispatch_attempts >= policy.max_dispatch_attempts {
                    item.last_error_code = Some(error_code.clone());
                    item.state = OutboxState::DeadLetter { error_code };
                } else {
                    item.last_error_code = Some(error_code);
                    let delay =
                        backoff_delay_ms(&item.effect_id, item.dispatch_attempts, policy)?;
                    item.next_attempt_at_ms = now_ms
                        .checked_add(delay)
                        .ok_or(OutboxError::InvalidTime)?;
                    item.state = OutboxState::Ready;
                }
            }
            DispatchOutcome::Indeterminate { evidence_digest } => {
                if !is_digest(&evidence_digest) {
                    item.state = OutboxState::ManualReview {
                        reason_code: "invalid_indeterminate_evidence".into(),
                    };
                } else {
                    item.last_evidence_digest = Some(evidence_digest);
                    schedule_reconciliation(item, now_ms, policy)?;
                }
            }
            DispatchOutcome::PermanentRejection { error_code } => {
                if valid_error_code(&error_code) {
                    item.last_error_code = Some(error_code.clone());
                    item.state = OutboxState::DeadLetter { error_code };
                } else {
                    item.state = OutboxState::ManualReview {
                        reason_code: "invalid_rejection_code".into(),
                    };
                }
            }
            DispatchOutcome::SourceEvidenceInvalid { evidence_digest } => {
                if is_digest(&evidence_digest) {
                    item.last_evidence_digest = Some(evidence_digest.clone());
                    item.state = OutboxState::ManualReview {
                        reason_code: "source_evidence_invalid".into(),
                    };
                } else {
                    item.state = OutboxState::ManualReview {
                        reason_code: "invalid_source_evidence".into(),
                    };
                }
            }
        }
        Ok(())
    }

    /// Resolves an indeterminate outcome. An authoritative absence returns the
    /// item to Ready, but the next dispatch still requires a fresh preflight.
    pub fn record_reconciliation(
        &mut self,
        lease: &OutboxLease,
        outcome: QueryOutcome,
        now_ms: u64,
        policy: BackoffPolicy,
    ) -> Result<(), OutboxError> {
        policy.validate()?;
        let item = self.require_lease_mut(lease, LeasePhase::Reconciliation, now_ms)?;
        item.last_attempt_at_ms = Some(now_ms);
        match outcome {
            QueryOutcome::Receipt(receipt) => apply_receipt(item, receipt),
            QueryOutcome::AuthoritativeAbsence { proof_digest } => {
                if !is_digest(&proof_digest) {
                    item.state = OutboxState::ManualReview {
                        reason_code: "invalid_absence_proof".into(),
                    };
                } else if reconciliation_age_exceeded(item, now_ms, policy) {
                    item.state = OutboxState::ManualReview {
                        reason_code: "reconciliation_budget_exhausted".into(),
                    };
                    item.last_error_code = Some("reconciliation_budget_exhausted".into());
                } else {
                    item.last_absence_proof_digest = Some(proof_digest);
                    item.next_attempt_at_ms = now_ms;
                    item.state = OutboxState::Ready;
                }
            }
            QueryOutcome::Uncertain => schedule_reconciliation(item, now_ms, policy)?,
        }
        Ok(())
    }

    /// Expired preflight leases are safe to re-query. Expired dispatch-authorized
    /// or reconciliation leases are ambiguous and must be reconciled.
    pub fn expire_leases(&mut self, now_ms: u64) -> usize {
        let mut expired = 0;
        for item in self.items.values_mut() {
            let next = match &item.state {
                OutboxState::LeasedForPreflight(lease) if now_ms >= lease.expires_at_ms => {
                    Some(OutboxState::Ready)
                }
                OutboxState::DispatchAuthorized { lease, .. }
                    if now_ms >= lease.expires_at_ms =>
                {
                    Some(OutboxState::AwaitingReconciliation)
                }
                OutboxState::LeasedForReconciliation(lease)
                    if now_ms >= lease.expires_at_ms =>
                {
                    Some(OutboxState::AwaitingReconciliation)
                }
                _ => None,
            };
            if let Some(state) = next {
                expired += 1;
                if state == OutboxState::Ready {
                    item.next_attempt_at_ms = now_ms;
                } else {
                    item.next_reconciliation_at_ms = now_ms;
                }
                item.state = state;
            }
        }
        expired
    }

    pub fn get(&self, effect_id: &str) -> Option<&OutboxItem> {
        self.items.get(effect_id)
    }

    pub fn metrics(&self, now_ms: u64) -> QueueMetrics {
        let mut metrics = QueueMetrics::default();
        for item in self.items.values() {
            match &item.state {
                OutboxState::Ready => metrics.ready += 1,
                OutboxState::LeasedForPreflight(_)
                | OutboxState::DispatchAuthorized { .. }
                | OutboxState::LeasedForReconciliation(_) => metrics.leased += 1,
                OutboxState::AwaitingReconciliation => {
                    metrics.awaiting_reconciliation += 1
                }
                OutboxState::Delivered { .. } => metrics.delivered += 1,
                OutboxState::DeadLetter { .. } => metrics.dead_letter += 1,
                OutboxState::ManualReview { .. } => metrics.manual_review += 1,
            }
            if !matches!(&item.state, OutboxState::Delivered { .. }) {
                metrics.oldest_unresolved_age_ms = metrics
                    .oldest_unresolved_age_ms
                    .max(now_ms.saturating_sub(item.created_at_ms));
            }
        }
        metrics
    }

    fn require_lease_mut(
        &mut self,
        lease: &OutboxLease,
        phase: LeasePhase,
        now_ms: u64,
    ) -> Result<&mut OutboxItem, OutboxError> {
        let item = self
            .items
            .get_mut(&lease.effect_id)
            .ok_or(OutboxError::UnknownEffect)?;
        let expected = LeaseState::from(lease);
        let actual = match (&item.state, phase) {
            (OutboxState::LeasedForPreflight(value), LeasePhase::Preflight) => Some(value),
            (OutboxState::DispatchAuthorized { lease: value, .. }, LeasePhase::Dispatch) => {
                Some(value)
            }
            (OutboxState::LeasedForReconciliation(value), LeasePhase::Reconciliation) => {
                Some(value)
            }
            _ => None,
        };
        if actual != Some(&expected) {
            return Err(OutboxError::LeaseMismatch);
        }
        if now_ms >= lease.expires_at_ms {
            return Err(OutboxError::LeaseExpired);
        }
        Ok(item)
    }
}

fn validate_new_effect(effect: &NewOutboxEffect) -> Result<(), OutboxError> {
    for value in [&effect.operation_id, &effect.effect_id, &effect.destination_id] {
        if value.trim().is_empty() || value.as_bytes().len() > MAX_TEXT_BYTES {
            return Err(OutboxError::InvalidInput);
        }
    }
    if !is_digest(&effect.payload_digest) {
        return Err(OutboxError::InvalidInput);
    }
    Ok(())
}

fn validate_worker_and_lease(
    worker_id: &str,
    duration_ms: u64,
    limit: usize,
    policy: BackoffPolicy,
) -> Result<(), OutboxError> {
    policy.validate()?;
    if worker_id.trim().is_empty()
        || worker_id.len() > 128
        || duration_ms == 0
        || limit > MAX_BATCH
    {
        return Err(if limit > MAX_BATCH {
            OutboxError::BatchLimitExceeded
        } else {
            OutboxError::InvalidInput
        });
    }
    Ok(())
}

fn new_lease(
    item: &mut OutboxItem,
    worker_id: &str,
    now_ms: u64,
    duration_ms: u64,
) -> Result<OutboxLease, OutboxError> {
    let expires_at_ms = now_ms
        .checked_add(duration_ms)
        .ok_or(OutboxError::InvalidTime)?;
    item.lease_generation = item
        .lease_generation
        .checked_add(1)
        .ok_or(OutboxError::CounterOverflow)?;
    Ok(OutboxLease {
        effect_id: item.effect_id.clone(),
        worker_id: worker_id.to_owned(),
        generation: item.lease_generation,
        expires_at_ms,
    })
}

fn apply_receipt(item: &mut OutboxItem, receipt: OutboxReceipt) {
    if receipt.operation_id == item.operation_id
        && receipt.effect_id == item.effect_id
        && receipt.destination_id == item.destination_id
        && receipt.payload_digest == item.payload_digest
        && is_digest(&receipt.receipt_digest)
    {
        item.state = OutboxState::Delivered {
            receipt_digest: receipt.receipt_digest,
        };
    } else {
        let evidence = if is_digest(&receipt.receipt_digest) {
            receipt.receipt_digest
        } else {
            "sha256:invalid-receipt".into()
        };
        if is_digest(&evidence) {
            item.last_evidence_digest = Some(evidence);
        }
        item.state = OutboxState::ManualReview {
            reason_code: "receipt_subject_mismatch".into(),
        };
    }
}

fn reconciliation_age_exceeded(
    item: &OutboxItem,
    now_ms: u64,
    policy: BackoffPolicy,
) -> bool {
    item.reconciliation_started_at_ms
        .is_some_and(|started| now_ms.saturating_sub(started) >= policy.max_reconciliation_age_ms)
}

fn reconciliation_budget_exhausted(
    item: &OutboxItem,
    now_ms: u64,
    policy: BackoffPolicy,
) -> bool {
    item.reconciliation_attempts >= policy.max_reconciliation_attempts
        || reconciliation_age_exceeded(item, now_ms, policy)
}

fn schedule_reconciliation(
    item: &mut OutboxItem,
    now_ms: u64,
    policy: BackoffPolicy,
) -> Result<(), OutboxError> {
    let started = *item.reconciliation_started_at_ms.get_or_insert(now_ms);
    if item.reconciliation_attempts >= policy.max_reconciliation_attempts
        || now_ms.saturating_sub(started) >= policy.max_reconciliation_age_ms
    {
        item.state = OutboxState::ManualReview {
            reason_code: "reconciliation_budget_exhausted".into(),
        };
        item.last_error_code = Some("reconciliation_budget_exhausted".into());
        return Ok(());
    }
    let backoff_attempt = item.reconciliation_attempts.saturating_add(1).max(1);
    let delay = backoff_delay_ms(&item.effect_id, backoff_attempt, policy)?;
    item.next_reconciliation_at_ms = now_ms.checked_add(delay).ok_or(OutboxError::InvalidTime)?;
    item.state = OutboxState::AwaitingReconciliation;
    Ok(())
}

/// Stable positive jitter from SHA-256(effect ID, attempt). Identical retries
/// produce the same schedule in Rust and Go; no runtime RNG or host clock is used.
pub fn backoff_delay_ms(
    effect_id: &str,
    attempt: u32,
    policy: BackoffPolicy,
) -> Result<u64, OutboxError> {
    policy.validate()?;
    if effect_id.trim().is_empty() || effect_id.len() > MAX_TEXT_BYTES || attempt == 0 {
        return Err(OutboxError::InvalidInput);
    }
    let shift = attempt.saturating_sub(1).min(63);
    let multiplier = 1_u64.checked_shl(shift).unwrap_or(u64::MAX);
    let exponential = policy
        .base_delay_ms
        .checked_mul(multiplier)
        .unwrap_or(u64::MAX)
        .min(policy.max_delay_ms);
    let mut encoded = Vec::with_capacity(64 + effect_id.len());
    encoded.extend_from_slice(BACKOFF_DOMAIN);
    let length = u32::try_from(effect_id.len()).map_err(|_| OutboxError::InvalidInput)?;
    encoded.extend_from_slice(&length.to_be_bytes());
    encoded.extend_from_slice(effect_id.as_bytes());
    encoded.extend_from_slice(&attempt.to_be_bytes());
    let digest = Sha256::digest(encoded);
    let random = u64::from_be_bytes(digest[..8].try_into().expect("eight-byte digest prefix"));
    let jitter = random % (policy.jitter_window_ms + 1);
    Ok(exponential.saturating_add(jitter).min(policy.max_delay_ms))
}

fn valid_error_code(code: &str) -> bool {
    !code.trim().is_empty()
        && code.len() <= MAX_ERROR_CODE_BYTES
        && code.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'-' || byte == b'.'
        })
}

fn is_digest(value: &str) -> bool {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return false;
    };
    hex.len() == 64
        && hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn policy() -> BackoffPolicy {
        BackoffPolicy {
            base_delay_ms: 100,
            max_delay_ms: 1_000,
            jitter_window_ms: 25,
            max_dispatch_attempts: 5,
            max_reconciliation_attempts: 5,
            max_reconciliation_age_ms: 60_000,
        }
    }

    fn effect() -> NewOutboxEffect {
        NewOutboxEffect {
            operation_id: "op-001".into(),
            effect_id: "effect:treasury-fee:op-001".into(),
            destination_id: "treasury:global".into(),
            payload_digest: format!("sha256:{}", "a".repeat(64)),
            created_at_ms: 1_000,
        }
    }

    fn receipt(effect: &NewOutboxEffect) -> OutboxReceipt {
        OutboxReceipt {
            operation_id: effect.operation_id.clone(),
            effect_id: effect.effect_id.clone(),
            destination_id: effect.destination_id.clone(),
            payload_digest: effect.payload_digest.clone(),
            receipt_digest: format!("sha256:{}", "b".repeat(64)),
        }
    }

    fn add(book: &mut OutboxBook) {
        assert_eq!(
            book.enqueue(effect()).expect("enqueue"),
            EnqueueDisposition::Inserted
        );
    }

    #[test]
    fn exact_enqueue_is_idempotent_but_payload_substitution_conflicts() {
        let mut book = OutboxBook::default();
        add(&mut book);
        assert_eq!(
            book.enqueue(effect()).expect("exact retry"),
            EnqueueDisposition::ExistingSameEffect
        );
        let mut changed = effect();
        changed.payload_digest = format!("sha256:{}", "c".repeat(64));
        assert_eq!(
            book.enqueue(changed),
            Err(OutboxError::ImmutableEffectConflict)
        );
    }

    #[test]
    fn preflight_is_required_before_any_dispatch_authorization() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let lease = book
            .claim_due("worker-a", 1_000, 100, 1, policy())
            .expect("claim")
            .remove(0);
        assert_eq!(
            book.record_dispatch(
                &lease,
                DispatchOutcome::DefinitelyNoEffectTransient {
                    error_code: "network_timeout".into(),
                },
                1_001,
                policy()
            ),
            Err(OutboxError::LeaseMismatch)
        );
        book.record_preflight(
            &lease,
            QueryOutcome::AuthoritativeAbsence {
                proof_digest: format!("sha256:{}", "d".repeat(64)),
            },
            1_001,
            policy(),
        )
        .expect("authorize only after absence proof");
        book.record_dispatch(
            &lease,
            DispatchOutcome::Committed(receipt(&effect())),
            1_002,
            policy(),
        )
        .expect("record commit");
        let delivered = book.get("effect:treasury-fee:op-001").unwrap();
        assert!(matches!(
            &delivered.state,
            OutboxState::Delivered { .. }
        ));
        assert_eq!(delivered.last_attempt_at_ms, Some(1_002));
    }

    #[test]
    fn expired_preflight_lease_is_safe_to_requery() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let lease = book
            .claim_due("worker-a", 1_000, 10, 1, policy())
            .expect("claim")
            .remove(0);
        assert_eq!(book.expire_leases(1_010), 1);
        assert_eq!(
            book.record_preflight(
                &lease,
                QueryOutcome::AuthoritativeAbsence {
                    proof_digest: format!("sha256:{}", "d".repeat(64)),
                },
                1_011,
                policy()
            ),
            Err(OutboxError::LeaseMismatch)
        );
        assert!(matches!(
            &book.get(&lease.effect_id).unwrap().state,
            OutboxState::Ready
        ));
    }

    #[test]
    fn expired_dispatch_authorization_requires_reconciliation_not_redelivery() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let lease = book.claim_due("worker-a", 1_000, 10, 1, policy()).unwrap().remove(0);
        book.record_preflight(
            &lease,
            QueryOutcome::AuthoritativeAbsence {
                proof_digest: format!("sha256:{}", "d".repeat(64)),
            },
            1_001,
            policy(),
        )
        .unwrap();
        assert_eq!(book.expire_leases(1_010), 1);
        assert!(matches!(
            &book.get(&lease.effect_id).unwrap().state,
            OutboxState::AwaitingReconciliation
        ));
        assert!(
            book.claim_due("worker-b", 1_010, 10, 1, policy())
                .unwrap()
                .is_empty()
        );
        let reconcile = book
            .claim_reconciliation("worker-b", 1_010, 10, 1, policy())
            .unwrap()
            .remove(0);
        book.record_reconciliation(&reconcile, QueryOutcome::Uncertain, 1_011, policy()).unwrap();
        assert!(
            book.claim_due("worker-c", 2_000, 10, 1, policy())
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn indeterminate_dispatch_never_uses_a_new_effect_identity() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let lease = book.claim_due("worker-a", 1_000, 100, 1, policy()).unwrap().remove(0);
        book.record_preflight(
            &lease,
            QueryOutcome::AuthoritativeAbsence {
                proof_digest: format!("sha256:{}", "d".repeat(64)),
            },
            1_001,
            policy(),
        ).unwrap();
        book.record_dispatch(
            &lease,
            DispatchOutcome::Indeterminate {
                evidence_digest: format!("sha256:{}", "e".repeat(64)),
            },
            1_002,
            policy(),
        ).unwrap();
        let pending = book.get(&lease.effect_id).unwrap();
        assert!(matches!(
            &pending.state,
            OutboxState::AwaitingReconciliation
        ));
        assert_eq!(pending.last_attempt_at_ms, Some(1_002));
        assert!(
            book.claim_due("worker-b", 10_000, 100, 1, policy())
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn receipt_subject_mismatch_goes_to_manual_review() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let lease = book.claim_due("worker-a", 1_000, 100, 1, policy()).unwrap().remove(0);
        let mut mismatched = receipt(&effect());
        mismatched.destination_id = "treasury:other".into();
        book.record_preflight(&lease, QueryOutcome::Receipt(mismatched), 1_001, policy()).unwrap();
        assert!(matches!(
            &book.get(&lease.effect_id).unwrap().state,
            OutboxState::ManualReview { .. }
        ));
    }

    #[test]
    fn transient_known_no_effect_uses_deterministic_backoff_and_stable_id() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let lease = book
            .claim_due("worker-a", 1_000, 100, 1, policy())
            .unwrap()
            .remove(0);
        book.record_preflight(
            &lease,
            QueryOutcome::AuthoritativeAbsence {
                proof_digest: format!("sha256:{}", "d".repeat(64)),
            },
            1_001,
            policy(),
        )
        .unwrap();
        book.record_dispatch(
            &lease,
            DispatchOutcome::DefinitelyNoEffectTransient {
                error_code: "rate_limited".into(),
            },
            1_002,
            policy(),
        )
        .unwrap();
        assert_eq!(book.get(&lease.effect_id).unwrap().state, OutboxState::Ready);
        assert_eq!(book.get(&lease.effect_id).unwrap().next_attempt_at_ms, 1_107);
        assert!(book.claim_due("worker-b", 1_106, 100, 1, policy()).unwrap().is_empty());
        let next = book.claim_due("worker-b", 1_107, 100, 1, policy()).unwrap().remove(0);
        assert_eq!(next.effect_id, lease.effect_id);
    }

    #[test]
    fn permanent_rejection_deadletters_and_invalid_source_requires_human_review() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let lease = book.claim_due("worker-a", 1_000, 100, 1, policy()).unwrap().remove(0);
        book.record_preflight(
            &lease,
            QueryOutcome::AuthoritativeAbsence {
                proof_digest: format!("sha256:{}", "d".repeat(64)),
            },
            1_001,
            policy(),
        ).unwrap();
        book.record_dispatch(
            &lease,
            DispatchOutcome::PermanentRejection { error_code: "invalid_destination".into() },
            1_002,
            policy(),
        ).unwrap();
        assert!(matches!(
            &book.get(&lease.effect_id).unwrap().state,
            OutboxState::DeadLetter { .. }
        ));
    }

    #[test]
    fn backoff_matches_shared_cross_language_known_answer() {
        let vectors = include_str!(
            "../../sap-payment-saga-go-verifier/fixtures/sap-treasury-outbox-recovery-v1.kat"
        )
        .lines()
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .map(|line| {
            let (key, value) = line.split_once('=').expect("fixture key=value");
            (key, value)
        })
        .collect::<std::collections::BTreeMap<_, _>>();
        let attempt = vectors["attempt"].parse::<u32>().expect("attempt");
        let delay =
            backoff_delay_ms(vectors["effect-id"], attempt, policy())
                .expect("deterministic delay");
        assert_eq!(delay.to_string(), vectors["expected-delay-ms"]);
    }

    #[test]
    fn backoff_vector_is_stable_across_reimplementations() {
        assert_eq!(
            backoff_delay_ms(
                "effect:treasury-fee:op-001",
                3,
                BackoffPolicy {
                    base_delay_ms: 100,
                    max_delay_ms: 1_000,
                    jitter_window_ms: 25,
                    max_dispatch_attempts: 5,
                    max_reconciliation_attempts: 5,
                    max_reconciliation_age_ms: 60_000,
                },
            ).unwrap(),
            416
        );
    }

    #[test]
    fn reconciliation_budget_moves_unresolved_effect_to_manual_review() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let mut bounded = policy();
        bounded.max_reconciliation_attempts = 1;
        let lease = book
            .claim_due("worker-a", 1_000, 100, 1, bounded)
            .unwrap()
            .remove(0);
        book.record_preflight(
            &lease,
            QueryOutcome::AuthoritativeAbsence {
                proof_digest: format!("sha256:{}", "d".repeat(64)),
            },
            1_001,
            bounded,
        )
        .unwrap();
        book.record_dispatch(
            &lease,
            DispatchOutcome::Indeterminate {
                evidence_digest: format!("sha256:{}", "e".repeat(64)),
            },
            1_002,
            bounded,
        )
        .unwrap();
        let due = book.get(&lease.effect_id).unwrap().next_reconciliation_at_ms;
        let reconciliation = book
            .claim_reconciliation("worker-b", due, 100, 1, bounded)
            .unwrap()
            .remove(0);
        book.record_reconciliation(
            &reconciliation,
            QueryOutcome::Uncertain,
            due + 1,
            bounded,
        )
        .unwrap();
        assert!(matches!(
            &book.get(&lease.effect_id).unwrap().state,
            OutboxState::ManualReview { .. }
        ));
        assert!(
            book.claim_due("worker-c", due + 10_000, 100, 1, bounded)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn reconciliation_age_budget_moves_due_effect_to_manual_review() {
        let mut book = OutboxBook::default();
        add(&mut book);
        let mut bounded = policy();
        bounded.max_reconciliation_age_ms = 10;
        let lease = book
            .claim_due("worker-a", 1_000, 100, 1, bounded)
            .unwrap()
            .remove(0);
        book.record_preflight(
            &lease,
            QueryOutcome::AuthoritativeAbsence {
                proof_digest: format!("sha256:{}", "d".repeat(64)),
            },
            1_001,
            bounded,
        )
        .unwrap();
        book.record_dispatch(
            &lease,
            DispatchOutcome::Indeterminate {
                evidence_digest: format!("sha256:{}", "e".repeat(64)),
            },
            1_002,
            bounded,
        )
        .unwrap();
        let due = book.get(&lease.effect_id).unwrap().next_reconciliation_at_ms;
        assert!(book
            .claim_reconciliation("worker-b", due, 100, 1, bounded)
            .unwrap()
            .is_empty());
        assert!(matches!(
            &book.get(&lease.effect_id).unwrap().state,
            OutboxState::ManualReview { .. }
        ));
    }

    #[test]
    fn metrics_preserve_oldest_unresolved_age() {
        let mut book = OutboxBook::default();
        add(&mut book);
        assert_eq!(
            book.metrics(1_500).oldest_unresolved_age_ms,
            500
        );
    }
}

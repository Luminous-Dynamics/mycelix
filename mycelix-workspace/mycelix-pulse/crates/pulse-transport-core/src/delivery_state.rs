//! Bounded delivery lifecycle and retry policy for a single logical message.
//!
//! This is a deterministic in-memory primitive, not a durable outbox. A caller
//! must persist/replay its accepted events in the application's trusted store
//! before claiming crash/restart durability. Adapter and receipt evidence must
//! be authenticated by their owning boundary before being passed here.

use std::collections::BTreeMap;

use crate::{
    user_visible_delivery_state, DeliveryEvidenceStage, UnixMillis,
    UserVisibleDeliveryState, VerifiedRecipientReceipt,
};

const MAX_MESSAGE_ID_BYTES: usize = 128;
const MAX_RECIPIENT_ID_BYTES: usize = 512;
const MAX_ATTEMPT_ID_BYTES: usize = 128;
const MAX_ATTEMPTS_HARD_LIMIT: u32 = 32;
const MAX_BACKOFF_MILLIS: u64 = 24 * 60 * 60 * 1000;

/// Exponential retry budget, bounded so malformed configuration cannot create
/// unbounded loops or timers. The attempt limit includes the first send.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RetryPolicy {
    max_attempts: u32,
    base_delay_millis: u64,
    max_delay_millis: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RetryPolicyError {
    ZeroAttempts,
    TooManyAttempts,
    ZeroBaseDelay,
    MaxDelayBelowBaseDelay,
    DelayExceedsHardLimit,
}

impl RetryPolicy {
    pub fn new(
        max_attempts: u32,
        base_delay_millis: u64,
        max_delay_millis: u64,
    ) -> Result<Self, RetryPolicyError> {
        if max_attempts == 0 {
            return Err(RetryPolicyError::ZeroAttempts);
        }
        if max_attempts > MAX_ATTEMPTS_HARD_LIMIT {
            return Err(RetryPolicyError::TooManyAttempts);
        }
        if base_delay_millis == 0 {
            return Err(RetryPolicyError::ZeroBaseDelay);
        }
        if max_delay_millis < base_delay_millis {
            return Err(RetryPolicyError::MaxDelayBelowBaseDelay);
        }
        if max_delay_millis > MAX_BACKOFF_MILLIS {
            return Err(RetryPolicyError::DelayExceedsHardLimit);
        }
        Ok(Self { max_attempts, base_delay_millis, max_delay_millis })
    }

    pub const fn max_attempts(self) -> u32 { self.max_attempts }
    pub const fn base_delay_millis(self) -> u64 { self.base_delay_millis }
    pub const fn max_delay_millis(self) -> u64 { self.max_delay_millis }

    fn delay_after(self, attempt_number: u32, message_id: &str, recipient_id: &str) -> u64 {
        let exponent = attempt_number.saturating_sub(1).min(63);
        let multiplier = 1_u64.checked_shl(exponent).unwrap_or(u64::MAX);
        let window = self.base_delay_millis
            .saturating_mul(multiplier)
            .min(self.max_delay_millis);

        // Deterministic, key-specific +/-25% jitter spreads retry timers without
        // making replay tests nondeterministic or requiring a weak PRNG.
        let spread = window / 4;
        let mut hash = 0xcbf29ce484222325_u64;
        for byte in message_id.bytes()
            .chain([0_u8])
            .chain(recipient_id.bytes())
            .chain(attempt_number.to_le_bytes())
        {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
        let jitter_width = spread.saturating_mul(2).saturating_add(1);
        window.saturating_sub(spread)
            .saturating_add(hash % jitter_width)
            .min(self.max_delay_millis)
    }
}

/// State is deliberately explicit about what evidence exists. In particular,
/// `OutcomeUnknown` is not `Failed`, and neither is `Delivered`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeliveryStatus {
    Queued,
    InFlight,
    AwaitingReceipt,
    RetryScheduled,
    /// No further attempt can fit before expiry; wait for expiry or a receipt.
    AwaitingDeadline,
    Delivered,
    Failed,
    Expired,
    Revoked,
    OutcomeUnknown,
}

impl DeliveryStatus {
    pub const fn user_visible(self) -> UserVisibleDeliveryState {
        match self {
            Self::Queued => UserVisibleDeliveryState::Queued,
            Self::InFlight | Self::AwaitingReceipt | Self::RetryScheduled
            | Self::AwaitingDeadline => UserVisibleDeliveryState::Pending,
            Self::Delivered => UserVisibleDeliveryState::Delivered,
            Self::Failed => UserVisibleDeliveryState::Failed,
            Self::Expired => UserVisibleDeliveryState::Expired,
            Self::Revoked => UserVisibleDeliveryState::Failed,
            Self::OutcomeUnknown => UserVisibleDeliveryState::Unknown,
        }
    }

    fn is_terminal(self) -> bool {
        matches!(self, Self::Delivered | Self::Failed | Self::Expired | Self::Revoked)
    }
}

/// Only reports from the adapter boundary; none of these proves recipient
/// delivery. A separately verified receipt is required for that transition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttemptOutcome {
    AcceptedByAdapter,
    LinkLayerAcknowledged,
    GatewayAcknowledged,
    TemporarilyUnavailable,
    PermanentlyRejected,
    OutcomeUnknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttemptStart {
    Started { attempt_number: u32 },
    AlreadyStarted { attempt_number: u32 },
    AlreadyCompleted { attempt_number: u32 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LifecycleError {
    InvalidMessageId,
    InvalidRecipientId,
    InvalidAttemptId,
    MessageExpired,
    TerminalState,
    RetryNotReady { at_unix_millis: u64 },
    AttemptLimitReached,
    NoMatchingActiveAttempt,
    ConflictingDuplicateOutcome,
    ReceiptBindingMismatch,
    ReceiptNotAcceptedInCurrentState,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Attempt {
    started_at: UnixMillis,
    outcome: Option<AttemptOutcome>,
}

/// Per-logical-message state. This map is bounded by the configured number of
/// attempts (at most 32) and refuses retry attempts after expiry or revocation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeliveryLifecycle {
    logical_message_id: String,
    recipient_id: String,
    expires_at: UnixMillis,
    retry_policy: RetryPolicy,
    status: DeliveryStatus,
    attempts: BTreeMap<String, Attempt>,
    active_attempt_id: Option<String>,
    next_attempt_at: Option<UnixMillis>,
    may_have_reached_recipient: bool,
}

impl DeliveryLifecycle {
    pub fn new(
        logical_message_id: impl Into<String>,
        recipient_id: impl Into<String>,
        expires_at: UnixMillis,
        retry_policy: RetryPolicy,
    ) -> Result<Self, LifecycleError> {
        let logical_message_id = logical_message_id.into();
        let recipient_id = recipient_id.into();
        if !valid_identifier(&logical_message_id, MAX_MESSAGE_ID_BYTES) {
            return Err(LifecycleError::InvalidMessageId);
        }
        if !valid_identifier(&recipient_id, MAX_RECIPIENT_ID_BYTES) {
            return Err(LifecycleError::InvalidRecipientId);
        }
        Ok(Self {
            logical_message_id,
            recipient_id,
            expires_at,
            retry_policy,
            status: DeliveryStatus::Queued,
            attempts: BTreeMap::new(),
            active_attempt_id: None,
            next_attempt_at: None,
            may_have_reached_recipient: false,
        })
    }

    pub fn logical_message_id(&self) -> &str { &self.logical_message_id }
    pub fn recipient_id(&self) -> &str { &self.recipient_id }
    pub const fn status(&self) -> DeliveryStatus { self.status }
    pub fn attempt_count(&self) -> u32 { self.attempts.len() as u32 }
    pub const fn next_attempt_at(&self) -> Option<UnixMillis> { self.next_attempt_at }
    pub const fn expires_at(&self) -> UnixMillis { self.expires_at }
    pub const fn retry_policy(&self) -> RetryPolicy { self.retry_policy }

    /// Begin an attempt with a caller-generated, stable attempt ID. Repeating
    /// the same ID is idempotent: it will not consume budget or start another
    /// send. Use the same logical message ID across all paths/retries so the
    /// receiver's own deduplication key remains stable.
    pub fn start_attempt(
        &mut self,
        attempt_id: impl Into<String>,
        now: UnixMillis,
    ) -> Result<AttemptStart, LifecycleError> {
        self.advance_time(now);
        let attempt_id = attempt_id.into();
        if !valid_identifier(&attempt_id, MAX_ATTEMPT_ID_BYTES) {
            return Err(LifecycleError::InvalidAttemptId);
        }
        if let Some(existing) = self.attempts.get(&attempt_id) {
            let attempt_number = self.attempt_number(&attempt_id);
            return Ok(if existing.outcome.is_some() {
                AttemptStart::AlreadyCompleted { attempt_number }
            } else {
                AttemptStart::AlreadyStarted { attempt_number }
            });
        }
        if self.status.is_terminal() {
            return Err(if self.status == DeliveryStatus::Expired {
                LifecycleError::MessageExpired
            } else {
                LifecycleError::TerminalState
            });
        }
        if self.status == DeliveryStatus::RetryScheduled {
            if let Some(retry_at) = self.next_attempt_at {
                if now < retry_at {
                    return Err(LifecycleError::RetryNotReady {
                        at_unix_millis: retry_at.as_millis(),
                    });
                }
            }
        } else if self.status != DeliveryStatus::Queued {
            return Err(LifecycleError::TerminalState);
        }
        if self.attempt_count() >= self.retry_policy.max_attempts() {
            self.status = if self.may_have_reached_recipient {
                DeliveryStatus::OutcomeUnknown
            } else {
                DeliveryStatus::Failed
            };
            self.next_attempt_at = None;
            return Err(LifecycleError::AttemptLimitReached);
        }

        let attempt_number = self.attempt_count() + 1;
        self.attempts.insert(attempt_id.clone(), Attempt {
            started_at: now,
            outcome: None,
        });
        self.active_attempt_id = Some(attempt_id);
        self.next_attempt_at = None;
        self.status = DeliveryStatus::InFlight;
        Ok(AttemptStart::Started { attempt_number })
    }

    /// Record one adapter observation. Exact duplicate observations are
    /// idempotent; conflicting observations for one attempt fail closed.
    pub fn record_outcome(
        &mut self,
        attempt_id: &str,
        outcome: AttemptOutcome,
        now: UnixMillis,
    ) -> Result<bool, LifecycleError> {
        self.advance_time(now);
        if let Some(attempt) = self.attempts.get(attempt_id) {
            if let Some(previous) = attempt.outcome {
                return if previous == outcome {
                    Ok(false)
                } else {
                    Err(LifecycleError::ConflictingDuplicateOutcome)
                };
            }
        }
        if self.status == DeliveryStatus::Expired {
            return Err(LifecycleError::MessageExpired);
        }
        if self.status.is_terminal() {
            return Err(LifecycleError::TerminalState);
        }
        if self.active_attempt_id.as_deref() != Some(attempt_id) {
            return Err(LifecycleError::NoMatchingActiveAttempt);
        }

        let attempt_number = self.attempt_number(attempt_id);
        let attempt = self.attempts.get_mut(attempt_id)
            .ok_or(LifecycleError::NoMatchingActiveAttempt)?;
        attempt.outcome = Some(outcome);
        self.active_attempt_id = None;

        match outcome {
            AttemptOutcome::AcceptedByAdapter
            | AttemptOutcome::LinkLayerAcknowledged
            | AttemptOutcome::GatewayAcknowledged
            | AttemptOutcome::OutcomeUnknown => {
                self.may_have_reached_recipient = true;
                self.schedule_retry_or_wait(now, attempt_number, true);
            }
            AttemptOutcome::TemporarilyUnavailable => {
                self.schedule_retry_or_wait(now, attempt_number, false);
            }
            AttemptOutcome::PermanentlyRejected => {
                self.next_attempt_at = None;
                self.status = if self.may_have_reached_recipient {
                    DeliveryStatus::OutcomeUnknown
                } else {
                    DeliveryStatus::Failed
                };
            }
        }
        Ok(true)
    }

    /// A receipt capability can be issued only by the existing trusted
    /// verifier. It must bind this exact message and recipient. Late receipts
    /// at/after expiry are rejected because the current token does not carry a
    /// separately verified recipient-receive timestamp.
    pub fn record_verified_receipt(
        &mut self,
        receipt: &VerifiedRecipientReceipt,
        now: UnixMillis,
    ) -> Result<bool, LifecycleError> {
        self.advance_time(now);
        if self.status == DeliveryStatus::Expired {
            return Err(LifecycleError::MessageExpired);
        }
        if self.status == DeliveryStatus::Revoked {
            return Err(LifecycleError::ReceiptNotAcceptedInCurrentState);
        }
        if user_visible_delivery_state(
            DeliveryEvidenceStage::OutcomeUnknown,
            &self.logical_message_id,
            &self.recipient_id,
            Some(receipt),
        ) != UserVisibleDeliveryState::Delivered {
            return Err(LifecycleError::ReceiptBindingMismatch);
        }
        if self.status == DeliveryStatus::Delivered {
            return Ok(false);
        }
        self.status = DeliveryStatus::Delivered;
        self.active_attempt_id = None;
        self.next_attempt_at = None;
        Ok(true)
    }

    /// Explicitly stop future retries after consent/policy revocation. This
    /// does not erase a delivery already established by a verified receipt.
    pub fn revoke(&mut self, now: UnixMillis) -> bool {
        self.advance_time(now);
        if self.status == DeliveryStatus::Delivered || self.status.is_terminal() {
            return false;
        }
        self.status = DeliveryStatus::Revoked;
        self.active_attempt_id = None;
        self.next_attempt_at = None;
        true
    }

    /// Advance expiry without starting a send. Expiry is a hard boundary:
    /// an item cannot be retried at or after the deadline.
    pub fn advance_time(&mut self, now: UnixMillis) -> DeliveryStatus {
        if !self.status.is_terminal() && now >= self.expires_at {
            self.status = DeliveryStatus::Expired;
            self.active_attempt_id = None;
            self.next_attempt_at = None;
        }
        self.status
    }

    fn schedule_retry_or_wait(
        &mut self,
        now: UnixMillis,
        attempt_number: u32,
        outcome_may_have_sent: bool,
    ) {
        if outcome_may_have_sent {
            self.status = DeliveryStatus::AwaitingReceipt;
        }
        if self.attempt_count() >= self.retry_policy.max_attempts() {
            self.next_attempt_at = None;
            self.status = if self.may_have_reached_recipient {
                DeliveryStatus::OutcomeUnknown
            } else {
                DeliveryStatus::Failed
            };
            return;
        }
        let delay = self.retry_policy.delay_after(
            attempt_number,
            &self.logical_message_id,
            &self.recipient_id,
        );
        let retry_at = now.as_millis().saturating_add(delay);
        if retry_at >= self.expires_at.as_millis() {
            self.next_attempt_at = None;
            self.status = DeliveryStatus::AwaitingDeadline;
        } else {
            self.next_attempt_at = Some(UnixMillis::new(retry_at));
            self.status = DeliveryStatus::RetryScheduled;
        }
    }

    fn attempt_number(&self, attempt_id: &str) -> u32 {
        self.attempts.keys().position(|id| id == attempt_id).unwrap_or(0) as u32 + 1
    }
}

fn valid_identifier(value: &str, maximum_bytes: usize) -> bool {
    !value.trim().is_empty()
        && value.len() <= maximum_bytes
        && !value.chars().any(char::is_control)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ReceiptVerificationError, TrustedRecipientReceiptVerifier};

    fn policy(max_attempts: u32, base: u64, max: u64) -> RetryPolicy {
        RetryPolicy::new(max_attempts, base, max).unwrap()
    }

    fn lifecycle(expires: u64, max_attempts: u32) -> DeliveryLifecycle {
        DeliveryLifecycle::new(
            "logical-message-1",
            "recipient-1",
            UnixMillis::new(expires),
            policy(max_attempts, 100, 1000),
        ).unwrap()
    }

    struct TestVerifier;

    impl TrustedRecipientReceiptVerifier for TestVerifier {
        fn verify(
            &self,
            bytes: &[u8],
            logical_message_id: &str,
            recipient_id: &str,
        ) -> Result<(), ReceiptVerificationError> {
            if bytes == b"verified"
                && logical_message_id == "logical-message-1"
                && recipient_id == "recipient-1"
            {
                Ok(())
            } else {
                Err(ReceiptVerificationError::InvalidReceipt)
            }
        }
    }

    fn verified_receipt() -> VerifiedRecipientReceipt {
        VerifiedRecipientReceipt::verify(
            &TestVerifier,
            b"verified",
            "logical-message-1",
            "recipient-1",
        ).unwrap()
    }

    #[test]
    fn retry_is_bounded_idempotent_and_not_early() {
        let mut item = lifecycle(10_000, 3);
        assert_eq!(
            item.start_attempt("attempt-1", UnixMillis::new(100)).unwrap(),
            AttemptStart::Started { attempt_number: 1 }
        );
        assert_eq!(
            item.start_attempt("attempt-1", UnixMillis::new(100)).unwrap(),
            AttemptStart::AlreadyStarted { attempt_number: 1 }
        );
        assert_eq!(item.attempt_count(), 1);
        assert_eq!(
            item.record_outcome(
                "attempt-1",
                AttemptOutcome::TemporarilyUnavailable,
                UnixMillis::new(100),
            ),
            Ok(true)
        );
        assert_eq!(
            item.record_outcome(
                "attempt-1",
                AttemptOutcome::TemporarilyUnavailable,
                UnixMillis::new(101),
            ),
            Ok(false)
        );
        assert_eq!(
            item.record_outcome(
                "attempt-1",
                AttemptOutcome::GatewayAcknowledged,
                UnixMillis::new(101),
            ),
            Err(LifecycleError::ConflictingDuplicateOutcome)
        );
        let retry_at = item.next_attempt_at().unwrap();
        assert!(retry_at.as_millis() > 100);
        assert_eq!(
            item.start_attempt("attempt-2", UnixMillis::new(100)),
            Err(LifecycleError::RetryNotReady {
                at_unix_millis: retry_at.as_millis()
            })
        );
        assert_eq!(item.attempt_count(), 1);
        assert!(matches!(
            item.start_attempt("attempt-2", retry_at).unwrap(),
            AttemptStart::Started { attempt_number: 2 }
        ));
    }

    #[test]
    fn adapter_ack_is_not_delivery_and_exhaustion_remains_unknown() {
        let mut item = lifecycle(100_000, 1);
        item.start_attempt("attempt-1", UnixMillis::new(1)).unwrap();
        item.record_outcome(
            "attempt-1",
            AttemptOutcome::GatewayAcknowledged,
            UnixMillis::new(2),
        ).unwrap();
        assert_eq!(item.status(), DeliveryStatus::OutcomeUnknown);
        assert_eq!(item.status().user_visible(), UserVisibleDeliveryState::Unknown);
        assert_ne!(item.status().user_visible(), UserVisibleDeliveryState::Delivered);
        assert_eq!(
            item.start_attempt("attempt-2", UnixMillis::new(500)),
            Err(LifecycleError::TerminalState)
        );
    }

    #[test]
    fn verified_receipt_requires_exact_message_and_recipient_and_is_idempotent() {
        let mut item = lifecycle(10_000, 3);
        item.start_attempt("attempt-1", UnixMillis::new(1)).unwrap();
        item.record_outcome(
            "attempt-1",
            AttemptOutcome::GatewayAcknowledged,
            UnixMillis::new(2),
        ).unwrap();
        assert_eq!(
            item.record_verified_receipt(&verified_receipt(), UnixMillis::new(3)),
            Ok(true)
        );
        assert_eq!(item.status(), DeliveryStatus::Delivered);
        assert_eq!(
            item.record_verified_receipt(&verified_receipt(), UnixMillis::new(4)),
            Ok(false)
        );
        assert_eq!(
            item.start_attempt("attempt-2", UnixMillis::new(4)),
            Err(LifecycleError::TerminalState)
        );

        let wrong = VerifiedRecipientReceipt::verify(
            &TestVerifier,
            b"verified",
            "other-message",
            "recipient-1",
        ).unwrap_err();
        assert_eq!(wrong, ReceiptVerificationError::InvalidReceipt);
    }

    #[test]
    fn expiry_and_revocation_block_future_sends() {
        let mut expired = lifecycle(100, 3);
        assert_eq!(
            expired.start_attempt("attempt-1", UnixMillis::new(100)),
            Err(LifecycleError::MessageExpired)
        );
        assert_eq!(expired.status(), DeliveryStatus::Expired);

        let mut revoked = lifecycle(1000, 3);
        assert!(revoked.revoke(UnixMillis::new(10)));
        assert!(!revoked.revoke(UnixMillis::new(11)));
        assert_eq!(
            revoked.start_attempt("attempt-1", UnixMillis::new(12)),
            Err(LifecycleError::TerminalState)
        );
    }

    #[test]
    fn retry_that_cannot_fit_before_deadline_is_not_scheduled() {
        let mut item = lifecycle(150, 3);
        item.start_attempt("attempt-1", UnixMillis::new(100)).unwrap();
        item.record_outcome(
            "attempt-1",
            AttemptOutcome::TemporarilyUnavailable,
            UnixMillis::new(100),
        ).unwrap();
        assert_eq!(item.status(), DeliveryStatus::AwaitingDeadline);
        assert_eq!(item.next_attempt_at(), None);
        assert_eq!(item.advance_time(UnixMillis::new(150)), DeliveryStatus::Expired);
    }

    #[test]
    fn retry_policy_rejects_unbounded_or_invalid_values() {
        assert_eq!(RetryPolicy::new(0, 10, 100), Err(RetryPolicyError::ZeroAttempts));
        assert_eq!(RetryPolicy::new(33, 10, 100), Err(RetryPolicyError::TooManyAttempts));
        assert_eq!(RetryPolicy::new(1, 0, 100), Err(RetryPolicyError::ZeroBaseDelay));
        assert_eq!(RetryPolicy::new(1, 100, 10), Err(RetryPolicyError::MaxDelayBelowBaseDelay));
        assert_eq!(
            RetryPolicy::new(1, 100, MAX_BACKOFF_MILLIS + 1),
            Err(RetryPolicyError::DelayExceedsHardLimit)
        );
    }
}

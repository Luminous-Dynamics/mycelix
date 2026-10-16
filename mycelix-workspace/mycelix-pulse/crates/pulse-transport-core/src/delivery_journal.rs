//! Storage-neutral delivery event contract and deterministic journal model.
//!
//! `InMemoryDeliveryJournal` is a reference model for contract tests only.
//! It is not durable and must never be used as evidence of restart safety.
//! A persistent backend must atomically enforce revision compare-and-swap,
//! event-ID uniqueness, immutable message identity, and worker fencing while
//! writing the event and its projection in one transaction.

use std::collections::BTreeMap;

use crate::{
    delivery_state::AttemptOutcome, ReceiptVerificationError,
    TrustedRecipientReceiptVerifier, UnixMillis, VerifiedRecipientReceipt,
};

const MAX_MESSAGE_ID_BYTES: usize = 128;
const MAX_RECIPIENT_ID_BYTES: usize = 512;
const MAX_EVENT_ID_BYTES: usize = 128;
const MAX_ATTEMPT_ID_BYTES: usize = 128;
const MAX_ATTEMPTS: usize = 32;

/// Stable logical identity plus a cryptographic commitment to the exact
/// protected envelope. The caller computes the digest with a versioned,
/// domain-separated scheme; this module never hashes plaintext.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct MessageIdentity {
    logical_message_id: String,
    recipient_id: String,
    envelope_commitment: [u8; 32],
}

impl MessageIdentity {
    pub fn new(
        logical_message_id: impl Into<String>,
        recipient_id: impl Into<String>,
        envelope_commitment: [u8; 32],
    ) -> Result<Self, JournalError> {
        let logical_message_id = logical_message_id.into();
        let recipient_id = recipient_id.into();
        if !valid_identifier(&logical_message_id, MAX_MESSAGE_ID_BYTES) {
            return Err(JournalError::InvalidMessageId);
        }
        if !valid_identifier(&recipient_id, MAX_RECIPIENT_ID_BYTES) {
            return Err(JournalError::InvalidRecipientId);
        }
        Ok(Self {
            logical_message_id,
            recipient_id,
            envelope_commitment,
        })
    }

    pub fn logical_message_id(&self) -> &str {
        &self.logical_message_id
    }

    pub fn recipient_id(&self) -> &str {
        &self.recipient_id
    }

    pub const fn envelope_commitment(&self) -> &[u8; 32] {
        &self.envelope_commitment
    }

    fn principal(&self) -> (String, String) {
        (self.logical_message_id.clone(), self.recipient_id.clone())
    }
}

/// The event actor identifies whether a transition came from system policy or
/// a dispatcher worker. Worker epoch comparison must be enforced atomically by
/// a durable backend; an old worker cannot update state after a newer epoch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JournalActor {
    System,
    Worker { epoch: u64 },
}

/// Authenticated receipt material plus a verifier-issued capability. A
/// persistent backend should store the encoded receipt and verifier version,
/// then rerun verification on replay instead of serializing a success flag.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedReceiptEvidence {
    encoded_receipt: Vec<u8>,
    verifier_version: String,
    receipt: VerifiedRecipientReceipt,
}

impl VerifiedReceiptEvidence {
    pub fn verify<V: TrustedRecipientReceiptVerifier>(
        verifier: &V,
        encoded_receipt: &[u8],
        expected_logical_message_id: &str,
        expected_recipient_id: &str,
        expected_envelope_commitment: [u8; 32],
        verifier_version: impl Into<String>,
    ) -> Result<Self, ReceiptVerificationError> {
        let verifier_version = verifier_version.into();
        if verifier_version.trim().is_empty()
            || verifier_version.len() > 64
            || verifier_version.chars().any(char::is_control)
        {
            return Err(ReceiptVerificationError::InvalidContext);
        }
        let receipt = VerifiedRecipientReceipt::verify(
            verifier,
            encoded_receipt,
            expected_logical_message_id,
            expected_recipient_id,
            expected_envelope_commitment,
        )?;
        Ok(Self {
            encoded_receipt: encoded_receipt.to_vec(),
            verifier_version,
            receipt,
        })
    }

    pub fn encoded_receipt(&self) -> &[u8] {
        &self.encoded_receipt
    }

    pub fn verifier_version(&self) -> &str {
        &self.verifier_version
    }

    fn matches(
        &self,
        logical_message_id: &str,
        recipient_id: &str,
        envelope_commitment: &[u8; 32],
    ) -> bool {
        self.receipt.matches(
            logical_message_id,
            recipient_id,
            envelope_commitment,
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JournalEventKind {
    /// First event for a logical message. Must precede every other event.
    Queued {
        expires_at: UnixMillis,
        policy_generation: u64,
    },
    /// System fencing token update. Epoch must strictly increase.
    WorkerEpochAdvanced {
        new_epoch: u64,
    },
    /// Reserve an attempt durably before calling a transport adapter.
    AttemptStarted {
        attempt_id: String,
    },
    /// Record only an authenticated adapter-layer observation. This is not a
    /// recipient receipt and never directly means Delivered.
    AttemptOutcomeRecorded {
        attempt_id: String,
        outcome: AttemptOutcome,
        retry_at: Option<UnixMillis>,
    },
    /// A receipt wait timed out, so the system explicitly authorizes a bounded
    /// retry schedule. Policy code remains responsible for the actual budget.
    RetryScheduled {
        not_before: UnixMillis,
    },
    PolicyRevoked {
        policy_generation: u64,
    },
    /// Explicit terminal expiry record; it must be timestamped at/after expiry.
    Expired,
    /// The token can only be obtained from the configured trusted verifier.
    /// Persistent implementations must save the signed receipt material and
    /// re-verify it on replay; this in-memory capability is not a disk format.
    RecipientReceiptVerified {
        evidence: VerifiedReceiptEvidence,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JournalEvent {
    event_id: String,
    identity: MessageIdentity,
    occurred_at: UnixMillis,
    actor: JournalActor,
    kind: JournalEventKind,
}

impl JournalEvent {
    pub fn new(
        event_id: impl Into<String>,
        identity: MessageIdentity,
        occurred_at: UnixMillis,
        actor: JournalActor,
        kind: JournalEventKind,
    ) -> Result<Self, JournalError> {
        let event_id = event_id.into();
        if !valid_identifier(&event_id, MAX_EVENT_ID_BYTES) {
            return Err(JournalError::InvalidEventId);
        }
        Ok(Self {
            event_id,
            identity,
            occurred_at,
            actor,
            kind,
        })
    }

    pub fn event_id(&self) -> &str {
        &self.event_id
    }

    pub const fn identity(&self) -> &MessageIdentity {
        &self.identity
    }

    pub const fn occurred_at(&self) -> UnixMillis {
        self.occurred_at
    }

    pub const fn actor(&self) -> JournalActor {
        self.actor
    }

    pub const fn kind(&self) -> &JournalEventKind {
        &self.kind
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DurableDeliveryStatus {
    Queued,
    InFlight,
    AwaitingReceipt,
    RetryScheduled,
    AwaitingDeadline,
    Delivered,
    Failed,
    Expired,
    Revoked,
    OutcomeUnknown,
}

impl DurableDeliveryStatus {
    fn is_terminal(self) -> bool {
        matches!(
            self,
            Self::Delivered | Self::Failed | Self::Expired | Self::Revoked
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JournalSnapshot {
    identity: MessageIdentity,
    revision: u64,
    status: DurableDeliveryStatus,
    expires_at: UnixMillis,
    policy_generation: u64,
    revoked: bool,
    worker_epoch: u64,
    attempts: BTreeMap<String, Option<AttemptOutcome>>,
    active_attempt_id: Option<String>,
    next_attempt_at: Option<UnixMillis>,
    may_have_reached_recipient: bool,
    last_event_id: String,
}

impl JournalSnapshot {
    pub const fn identity(&self) -> &MessageIdentity {
        &self.identity
    }

    pub const fn revision(&self) -> u64 {
        self.revision
    }

    pub const fn status(&self) -> DurableDeliveryStatus {
        self.status
    }

    pub const fn expires_at(&self) -> UnixMillis {
        self.expires_at
    }

    pub const fn policy_generation(&self) -> u64 {
        self.policy_generation
    }

    pub const fn revoked(&self) -> bool {
        self.revoked
    }

    pub const fn worker_epoch(&self) -> u64 {
        self.worker_epoch
    }

    pub fn attempt_count(&self) -> usize {
        self.attempts.len()
    }

    pub fn active_attempt_id(&self) -> Option<&str> {
        self.active_attempt_id.as_deref()
    }

    pub const fn next_attempt_at(&self) -> Option<UnixMillis> {
        self.next_attempt_at
    }

    pub const fn may_have_reached_recipient(&self) -> bool {
        self.may_have_reached_recipient
    }

    pub fn last_event_id(&self) -> &str {
        &self.last_event_id
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JournalEntry {
    revision: u64,
    event: JournalEvent,
}

impl JournalEntry {
    pub const fn revision(&self) -> u64 {
        self.revision
    }

    pub const fn event(&self) -> &JournalEvent {
        &self.event
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppendOutcome {
    Appended { revision: u64 },
    Duplicate { original_revision: u64 },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JournalError {
    InvalidMessageId,
    InvalidRecipientId,
    InvalidEventId,
    InvalidAttemptId,
    RevisionConflict { expected: u64, actual: u64 },
    RevisionExhausted,
    EventIdConflict,
    MessageIdentityConflict,
    MissingQueueAcceptance,
    InvalidTransition,
    StaleWorkerEpoch { expected: u64, actual: u64 },
    WorkerEpochMustIncrease,
    AttemptLimitReached,
    AttemptAlreadyExists,
    NoMatchingActiveAttempt,
    ConflictingDuplicateOutcome,
    RetryTimeInvalid,
    MessageExpired,
    TerminalState,
    ReceiptBindingMismatch,
}

impl std::fmt::Display for JournalError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for JournalError {}

/// Required contract for a durable implementation: append must compare the
/// expected stream revision, enforce global event-ID uniqueness and worker
/// fencing, and persist the event+snapshot atomically. Implementations should
/// back this with a database transaction / conditional write, not a read-then-
/// write sequence.
pub trait DeliveryJournal {
    fn append(
        &mut self,
        expected_revision: u64,
        event: JournalEvent,
    ) -> Result<AppendOutcome, JournalError>;

    fn snapshot(&self, identity: &MessageIdentity) -> Option<&JournalSnapshot>;

    fn history(&self, identity: &MessageIdentity) -> Option<&[JournalEntry]>;
}

/// Deterministic reference model used for contract tests. Its memory disappears
/// on process exit, so it cannot qualify a durable outbox or crash recovery.
#[derive(Default)]
pub struct InMemoryDeliveryJournal {
    streams: BTreeMap<MessageIdentity, (JournalSnapshot, Vec<JournalEntry>)>,
    event_index: BTreeMap<String, (MessageIdentity, JournalEvent, u64)>,
    identity_commitments: BTreeMap<(String, String), [u8; 32]>,
}

impl DeliveryJournal for InMemoryDeliveryJournal {
    fn append(
        &mut self,
        expected_revision: u64,
        event: JournalEvent,
    ) -> Result<AppendOutcome, JournalError> {
        if let Some((identity, prior, revision)) = self.event_index.get(event.event_id()) {
            return if identity == &event.identity && prior == &event {
                Ok(AppendOutcome::Duplicate {
                    original_revision: *revision,
                })
            } else {
                Err(JournalError::EventIdConflict)
            };
        }

        let principal = event.identity.principal();
        if let Some(commitment) = self.identity_commitments.get(&principal) {
            if commitment != event.identity.envelope_commitment() {
                return Err(JournalError::MessageIdentityConflict);
            }
        }

        let actual_revision = self
            .streams
            .get(&event.identity)
            .map(|(snapshot, _)| snapshot.revision)
            .unwrap_or(0);
        if expected_revision != actual_revision {
            return Err(JournalError::RevisionConflict {
                expected: expected_revision,
                actual: actual_revision,
            });
        }

        let mut next = match self.streams.get(&event.identity) {
            Some((snapshot, _)) => {
                let mut next = snapshot.clone();
                apply_event(&mut next, &event)?;
                next
            }
            // initial_snapshot validates the first QueueAccepted event; do not
            // apply the same event a second time to an already-created stream.
            None => initial_snapshot(&event)?,
        };
        next.revision = actual_revision
            .checked_add(1)
            .ok_or(JournalError::RevisionExhausted)?;
        next.last_event_id = event.event_id.clone();

        let revision = next.revision;
        let stream = self
            .streams
            .entry(event.identity.clone())
            .or_insert_with(|| (next.clone(), Vec::new()));
        stream.0 = next;
        stream.1.push(JournalEntry {
            revision,
            event: event.clone(),
        });

        self.identity_commitments
            .entry(principal)
            .or_insert(*event.identity.envelope_commitment());
        self.event_index.insert(
            event.event_id.clone(),
            (event.identity.clone(), event, revision),
        );
        Ok(AppendOutcome::Appended { revision })
    }

    fn snapshot(&self, identity: &MessageIdentity) -> Option<&JournalSnapshot> {
        self.streams.get(identity).map(|(snapshot, _)| snapshot)
    }

    fn history(&self, identity: &MessageIdentity) -> Option<&[JournalEntry]> {
        self.streams
            .get(identity)
            .map(|(_, history)| history.as_slice())
    }
}

fn initial_snapshot(event: &JournalEvent) -> Result<JournalSnapshot, JournalError> {
    let JournalEventKind::Queued {
        expires_at,
        policy_generation,
    } = &event.kind
    else {
        return Err(JournalError::MissingQueueAcceptance);
    };
    if event.actor != JournalActor::System || event.occurred_at >= *expires_at {
        return Err(JournalError::InvalidTransition);
    }
    Ok(JournalSnapshot {
        identity: event.identity.clone(),
        revision: 0,
        status: DurableDeliveryStatus::Queued,
        expires_at: *expires_at,
        policy_generation: *policy_generation,
        revoked: false,
        worker_epoch: 0,
        attempts: BTreeMap::new(),
        active_attempt_id: None,
        next_attempt_at: None,
        may_have_reached_recipient: false,
        last_event_id: String::new(),
    })
}

fn apply_event(snapshot: &mut JournalSnapshot, event: &JournalEvent) -> Result<(), JournalError> {
    if snapshot.status == DurableDeliveryStatus::Expired {
        return Err(JournalError::MessageExpired);
    }
    if event.occurred_at >= snapshot.expires_at
        && !matches!(&event.kind, JournalEventKind::Expired)
    {
        return Err(JournalError::MessageExpired);
    }
    if snapshot.status == DurableDeliveryStatus::Revoked
        || snapshot.status == DurableDeliveryStatus::Failed
    {
        return Err(JournalError::TerminalState);
    }

    match &event.kind {
        JournalEventKind::Queued { .. } => Err(JournalError::InvalidTransition),
        JournalEventKind::WorkerEpochAdvanced { new_epoch } => {
            if event.actor != JournalActor::System || snapshot.status.is_terminal() {
                return Err(JournalError::InvalidTransition);
            }
            if *new_epoch <= snapshot.worker_epoch {
                return Err(JournalError::WorkerEpochMustIncrease);
            }
            // A takeover fences the old worker. If it had an active attempt,
            // recovery must treat it as possibly sent rather than lose the
            // attempt or accept a later callback from the stale epoch.
            if let Some(attempt_id) = snapshot.active_attempt_id.take() {
                snapshot.attempts.insert(
                    attempt_id,
                    Some(AttemptOutcome::OutcomeUnknown),
                );
                snapshot.may_have_reached_recipient = true;
                snapshot.status = DurableDeliveryStatus::OutcomeUnknown;
                snapshot.next_attempt_at = None;
            }
            snapshot.worker_epoch = *new_epoch;
            Ok(())
        }
        JournalEventKind::AttemptStarted { attempt_id } => {
            if event.occurred_at >= snapshot.expires_at {
                return Err(JournalError::MessageExpired);
            }
            let JournalActor::Worker { epoch } = event.actor else {
                return Err(JournalError::InvalidTransition);
            };
            if epoch != snapshot.worker_epoch {
                return Err(JournalError::StaleWorkerEpoch {
                    expected: snapshot.worker_epoch,
                    actual: epoch,
                });
            }
            if snapshot.revoked {
                return Err(JournalError::TerminalState);
            }
            if snapshot.status.is_terminal() {
                return Err(JournalError::TerminalState);
            }
            if !valid_identifier(attempt_id, MAX_ATTEMPT_ID_BYTES) {
                return Err(JournalError::InvalidAttemptId);
            }
            if snapshot.attempts.contains_key(attempt_id) {
                return Err(JournalError::AttemptAlreadyExists);
            }
            if snapshot.attempts.len() >= MAX_ATTEMPTS {
                return Err(JournalError::AttemptLimitReached);
            }
            match snapshot.status {
                DurableDeliveryStatus::Queued => {}
                DurableDeliveryStatus::RetryScheduled => {
                    let Some(not_before) = snapshot.next_attempt_at else {
                        return Err(JournalError::InvalidTransition);
                    };
                    if event.occurred_at < not_before {
                        return Err(JournalError::RetryTimeInvalid);
                    }
                }
                _ => return Err(JournalError::InvalidTransition),
            }
            snapshot.attempts.insert(attempt_id.clone(), None);
            snapshot.active_attempt_id = Some(attempt_id.clone());
            snapshot.next_attempt_at = None;
            snapshot.status = DurableDeliveryStatus::InFlight;
            Ok(())
        }
        JournalEventKind::AttemptOutcomeRecorded {
            attempt_id,
            outcome,
            retry_at,
        } => {
            let JournalActor::Worker { epoch } = event.actor else {
                return Err(JournalError::InvalidTransition);
            };
            if epoch != snapshot.worker_epoch {
                return Err(JournalError::StaleWorkerEpoch {
                    expected: snapshot.worker_epoch,
                    actual: epoch,
                });
            }
            if snapshot.active_attempt_id.as_deref() != Some(attempt_id.as_str()) {
                return Err(JournalError::NoMatchingActiveAttempt);
            }
            if snapshot.attempts.get(attempt_id).is_some_and(|outcome| outcome.is_some()) {
                return Err(JournalError::ConflictingDuplicateOutcome);
            }
            if event.occurred_at >= snapshot.expires_at {
                return Err(JournalError::MessageExpired);
            }
            match outcome {
                AttemptOutcome::TemporarilyUnavailable => {
                    let Some(not_before) = retry_at else {
                        return Err(JournalError::RetryTimeInvalid);
                    };
                    if not_before <= event.occurred_at {
                        return Err(JournalError::RetryTimeInvalid);
                    }
                    snapshot.next_attempt_at = if not_before < snapshot.expires_at {
                        Some(*not_before)
                    } else {
                        None
                    };
                    snapshot.status = if snapshot.next_attempt_at.is_some() {
                        DurableDeliveryStatus::RetryScheduled
                    } else {
                        DurableDeliveryStatus::AwaitingDeadline
                    };
                }
                _ => {
                    if retry_at.is_some() {
                        return Err(JournalError::InvalidTransition);
                    }
                    snapshot.next_attempt_at = None;
                    match outcome {
                        AttemptOutcome::AcceptedByAdapter
                        | AttemptOutcome::LinkLayerAcknowledged
                        | AttemptOutcome::GatewayAcknowledged
                        | AttemptOutcome::OutcomeUnknown => {
                            snapshot.may_have_reached_recipient = true;
                            snapshot.status = DurableDeliveryStatus::AwaitingReceipt;
                        }
                        AttemptOutcome::PermanentlyRejected => {
                            snapshot.status = if snapshot.may_have_reached_recipient {
                                DurableDeliveryStatus::OutcomeUnknown
                            } else {
                                DurableDeliveryStatus::Failed
                            };
                        }
                        AttemptOutcome::TemporarilyUnavailable => {
                            return Err(JournalError::InvalidTransition);
                        }
                    }
                }
            }
            snapshot.attempts.insert(attempt_id.clone(), Some(*outcome));
            snapshot.active_attempt_id = None;
            Ok(())
        }
        JournalEventKind::RetryScheduled { not_before } => {
            if event.actor != JournalActor::System
                || snapshot.revoked
                || snapshot.active_attempt_id.is_some()
                || !matches!(
                    snapshot.status,
                    DurableDeliveryStatus::AwaitingReceipt
                        | DurableDeliveryStatus::OutcomeUnknown
                )
                || *not_before <= event.occurred_at
            {
                return Err(JournalError::InvalidTransition);
            }
            if *not_before >= snapshot.expires_at {
                snapshot.next_attempt_at = None;
                snapshot.status = DurableDeliveryStatus::AwaitingDeadline;
            } else {
                snapshot.next_attempt_at = Some(*not_before);
                snapshot.status = DurableDeliveryStatus::RetryScheduled;
            }
            Ok(())
        }
        JournalEventKind::PolicyRevoked {
            policy_generation,
        } => {
            if event.actor != JournalActor::System
                || *policy_generation <= snapshot.policy_generation
                || snapshot.revoked
                || snapshot.status.is_terminal()
            {
                return Err(JournalError::InvalidTransition);
            }
            snapshot.policy_generation = *policy_generation;
            snapshot.revoked = true;
            if let Some(attempt_id) = snapshot.active_attempt_id.take() {
                // The send may already have crossed the network boundary.
                // Mark it ambiguous so a verified receipt can still resolve
                // the outcome, but never permit a further attempt.
                snapshot.attempts.insert(
                    attempt_id,
                    Some(AttemptOutcome::OutcomeUnknown),
                );
                snapshot.may_have_reached_recipient = true;
            }
            snapshot.next_attempt_at = None;
            snapshot.status = if snapshot.may_have_reached_recipient {
                DurableDeliveryStatus::OutcomeUnknown
            } else {
                DurableDeliveryStatus::Revoked
            };
            Ok(())
        }
        JournalEventKind::Expired => {
            if event.actor != JournalActor::System
                || event.occurred_at < snapshot.expires_at
                || snapshot.status.is_terminal()
            {
                return Err(JournalError::InvalidTransition);
            }
            snapshot.active_attempt_id = None;
            snapshot.next_attempt_at = None;
            snapshot.status = DurableDeliveryStatus::Expired;
            Ok(())
        }
        JournalEventKind::RecipientReceiptVerified { evidence } => {
            if event.actor != JournalActor::System {
                return Err(JournalError::InvalidTransition);
            }
            if event.occurred_at >= snapshot.expires_at {
                return Err(JournalError::MessageExpired);
            }
            if !evidence.matches(
                snapshot.identity.logical_message_id(),
                snapshot.identity.recipient_id(),
                snapshot.identity.envelope_commitment(),
            ) {
                return Err(JournalError::ReceiptBindingMismatch);
            }
            if snapshot.status == DurableDeliveryStatus::Delivered {
                return Ok(());
            }
            snapshot.status = DurableDeliveryStatus::Delivered;
            snapshot.active_attempt_id = None;
            snapshot.next_attempt_at = None;
            Ok(())
        }
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

    const ZERO_COMMITMENT: [u8; 32] = [0; 32];

    struct TestVerifier;

    impl TrustedRecipientReceiptVerifier for TestVerifier {
        fn verify(
            &self,
            bytes: &[u8],
            logical_message_id: &str,
            recipient_id: &str,
            envelope_commitment: &[u8; 32],
        ) -> Result<(), ReceiptVerificationError> {
            if bytes == b"receipt-ok"
                && logical_message_id == "message-1"
                && recipient_id == "recipient-1"
                && envelope_commitment == &ZERO_COMMITMENT
            {
                Ok(())
            } else {
                Err(ReceiptVerificationError::InvalidReceipt)
            }
        }
    }

    fn identity() -> MessageIdentity {
        MessageIdentity::new("message-1", "recipient-1", ZERO_COMMITMENT).unwrap()
    }

    fn event(
        event_id: &str,
        identity: MessageIdentity,
        time: u64,
        actor: JournalActor,
        kind: JournalEventKind,
    ) -> JournalEvent {
        JournalEvent::new(event_id, identity, UnixMillis::new(time), actor, kind).unwrap()
    }

    fn queued(journal: &mut InMemoryDeliveryJournal) -> MessageIdentity {
        let identity = identity();
        let initial = event(
            "event-queue",
            identity.clone(),
            1,
            JournalActor::System,
            JournalEventKind::Queued {
                expires_at: UnixMillis::new(10_000),
                policy_generation: 1,
            },
        );
        assert_eq!(
            journal.append(0, initial).unwrap(),
            AppendOutcome::Appended { revision: 1 }
        );
        identity
    }

    #[test]
    fn append_is_compare_and_swap_and_exact_replay_is_idempotent() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = identity();
        let first = event(
            "event-queue",
            identity.clone(),
            1,
            JournalActor::System,
            JournalEventKind::Queued {
                expires_at: UnixMillis::new(10_000),
                policy_generation: 1,
            },
        );
        assert_eq!(
            journal.append(0, first.clone()).unwrap(),
            AppendOutcome::Appended { revision: 1 }
        );
        assert_eq!(
            journal.append(0, first.clone()).unwrap(),
            AppendOutcome::Duplicate { original_revision: 1 }
        );
        assert_eq!(
            journal.append(0, event(
                "event-stale",
                identity.clone(),
                2,
                JournalActor::System,
                JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
            )),
            Err(JournalError::RevisionConflict { expected: 0, actual: 1 })
        );
        assert_eq!(journal.snapshot(&identity).unwrap().revision(), 1);
    }

    #[test]
    fn reusing_event_id_with_different_content_is_rejected() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        let conflicting = event(
            "event-queue",
            identity,
            2,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
        );
        assert_eq!(
            journal.append(1, conflicting),
            Err(JournalError::EventIdConflict)
        );
    }

    #[test]
    fn identity_commitment_cannot_change_for_same_message_and_recipient() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        let changed = MessageIdentity::new(
            "message-1",
            "recipient-1",
            [9; 32],
        ).unwrap();
        assert_eq!(
            journal.append(1, event(
                "event-substitute",
                changed,
                2,
                JournalActor::System,
                JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
            )),
            Err(JournalError::MessageIdentityConflict)
        );
        assert_eq!(journal.snapshot(&identity).unwrap().revision(), 1);
    }

    #[test]
    fn stale_worker_epoch_cannot_start_or_complete_an_attempt() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        journal.append(1, event(
            "event-epoch-1",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
        )).unwrap();

        assert_eq!(
            journal.append(2, event(
                "event-attempt-stale",
                identity.clone(),
                3,
                JournalActor::Worker { epoch: 0 },
                JournalEventKind::AttemptStarted { attempt_id: "attempt-1".into() },
            )),
            Err(JournalError::StaleWorkerEpoch { expected: 1, actual: 0 })
        );

        journal.append(2, event(
            "event-attempt",
            identity.clone(),
            3,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptStarted { attempt_id: "attempt-1".into() },
        )).unwrap();

        journal.append(3, event(
            "event-epoch-2",
            identity.clone(),
            4,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 2 },
        )).unwrap();

        assert_eq!(
            journal.append(4, event(
                "event-outcome-stale",
                identity.clone(),
                5,
                JournalActor::Worker { epoch: 1 },
                JournalEventKind::AttemptOutcomeRecorded {
                    attempt_id: "attempt-1".into(),
                    outcome: AttemptOutcome::GatewayAcknowledged,
                    retry_at: None,
                },
            )),
            Err(JournalError::StaleWorkerEpoch { expected: 2, actual: 1 })
        );
        assert_eq!(
            journal.snapshot(&identity).unwrap().status(),
            DurableDeliveryStatus::OutcomeUnknown
        );
    }

    #[test]
    fn verified_receipt_is_required_and_must_bind_commitment_identity_scope() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        let verifier = TestVerifier;
        let evidence = VerifiedReceiptEvidence::verify(
            &verifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();

        journal.append(1, event(
            "event-receipt",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::RecipientReceiptVerified { evidence },
        )).unwrap();
        assert_eq!(
            journal.snapshot(&identity).unwrap().status(),
            DurableDeliveryStatus::Delivered
        );
        assert_eq!(journal.snapshot(&identity).unwrap().revision(), 2);
    }

    #[test]
    fn retry_cannot_start_early_or_at_expiry() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        journal.append(1, event(
            "event-epoch",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
        )).unwrap();
        journal.append(2, event(
            "event-attempt",
            identity.clone(),
            3,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptStarted { attempt_id: "attempt-1".into() },
        )).unwrap();
        journal.append(3, event(
            "event-temp-unavailable",
            identity.clone(),
            4,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptOutcomeRecorded {
                attempt_id: "attempt-1".into(),
                outcome: AttemptOutcome::TemporarilyUnavailable,
                retry_at: Some(UnixMillis::new(100)),
            },
        )).unwrap();

        assert_eq!(
            journal.append(4, event(
                "event-early-retry",
                identity.clone(),
                99,
                JournalActor::Worker { epoch: 1 },
                JournalEventKind::AttemptStarted { attempt_id: "attempt-2".into() },
            )),
            Err(JournalError::RetryTimeInvalid)
        );
        assert_eq!(
            journal.append(4, event(
                "event-at-expiry",
                identity.clone(),
                10_000,
                JournalActor::Worker { epoch: 1 },
                JournalEventKind::AttemptStarted { attempt_id: "attempt-2".into() },
            )),
            Err(JournalError::MessageExpired)
        );
        assert_eq!(journal.snapshot(&identity).unwrap().revision(), 4);
    }

    #[test]
    fn terminal_expiry_rejects_late_receipt() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        journal.append(1, event(
            "event-expired",
            identity.clone(),
            10_000,
            JournalActor::System,
            JournalEventKind::Expired,
        )).unwrap();

        let evidence = VerifiedReceiptEvidence::verify(
            &TestVerifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();
        assert_eq!(
            journal.append(2, event(
                "event-late-receipt",
                identity.clone(),
                10_001,
                JournalActor::System,
                JournalEventKind::RecipientReceiptVerified { evidence },
            )),
            Err(JournalError::MessageExpired)
        );
        assert_eq!(
            journal.snapshot(&identity).unwrap().status(),
            DurableDeliveryStatus::Expired
        );
    }

    #[test]
    fn revoked_message_cannot_be_retried() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        journal.append(1, event(
            "event-revoke",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::PolicyRevoked { policy_generation: 2 },
        )).unwrap();
        assert_eq!(
            journal.append(2, event(
                "event-attempt-after-revoke",
                identity.clone(),
                3,
                JournalActor::Worker { epoch: 0 },
                JournalEventKind::AttemptStarted { attempt_id: "attempt-1".into() },
            )),
            Err(JournalError::TerminalState)
        );
    }

    #[test]
    fn revocation_fences_retry_without_erasing_an_inflight_receipt() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        journal.append(1, event(
            "event-epoch",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
        )).unwrap();
        journal.append(2, event(
            "event-attempt",
            identity.clone(),
            3,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptStarted { attempt_id: "attempt-1".into() },
        )).unwrap();
        journal.append(3, event(
            "event-revoke-inflight",
            identity.clone(),
            4,
            JournalActor::System,
            JournalEventKind::PolicyRevoked { policy_generation: 2 },
        )).unwrap();
        assert!(journal.snapshot(&identity).unwrap().revoked());
        assert_eq!(
            journal.snapshot(&identity).unwrap().status(),
            DurableDeliveryStatus::OutcomeUnknown
        );
        assert_eq!(
            journal.append(4, event(
                "event-attempt-after-revoke",
                identity.clone(),
                5,
                JournalActor::Worker { epoch: 1 },
                JournalEventKind::AttemptStarted { attempt_id: "attempt-2".into() },
            )),
            Err(JournalError::TerminalState)
        );

        let evidence = VerifiedReceiptEvidence::verify(
            &TestVerifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();
        journal.append(4, event(
            "event-receipt-after-revoke",
            identity.clone(),
            6,
            JournalActor::System,
            JournalEventKind::RecipientReceiptVerified { evidence },
        )).unwrap();
        assert_eq!(
            journal.snapshot(&identity).unwrap().status(),
            DurableDeliveryStatus::Delivered
        );
    }

    #[test]
    fn adapter_ack_never_maps_to_delivered_in_journal_projection() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        journal.append(1, event(
            "event-epoch",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
        )).unwrap();
        journal.append(2, event(
            "event-attempt",
            identity.clone(),
            3,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptStarted { attempt_id: "attempt-1".into() },
        )).unwrap();
        journal.append(3, event(
            "event-ack",
            identity.clone(),
            4,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptOutcomeRecorded {
                attempt_id: "attempt-1".into(),
                outcome: AttemptOutcome::GatewayAcknowledged,
                retry_at: None,
            },
        )).unwrap();
        assert_eq!(
            journal.snapshot(&identity).unwrap().status(),
            DurableDeliveryStatus::AwaitingReceipt
        );
    }
}

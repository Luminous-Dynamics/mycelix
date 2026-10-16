//! Storage-neutral delivery event contract and deterministic journal model.
//!
//! `InMemoryDeliveryJournal` is a reference model for contract tests only.
//! It is not durable and must never be used as evidence of restart safety.
//! A persistent backend must atomically enforce revision compare-and-swap,
//! event-ID uniqueness, immutable message identity, and worker fencing while
//! writing the event and its projection in one transaction.

use std::collections::BTreeMap;

use crate::MAX_RECEIPT_BYTES;

use crate::{
    delivery_state::AttemptOutcome, ReceiptVerificationError,
    TrustedRecipientReceiptVerifier, UnixMillis, VerifiedRecipientReceipt,
};

const MAX_MESSAGE_ID_BYTES: usize = 128;
const MAX_RECIPIENT_ID_BYTES: usize = 512;
const MAX_EVENT_ID_BYTES: usize = 128;
const MAX_ATTEMPT_ID_BYTES: usize = 128;
const MAX_ATTEMPTS: usize = 32;
/// Bound retained contract events per logical message; a backend may compact only
/// after preserving the evidence and replay guarantees required by policy.
const MAX_EVENTS_PER_MESSAGE: usize = 256;

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

/// A storage-facing event representation. Unlike `JournalEvent`, receipt
/// variants contain raw authenticated bytes and a verifier-version label,
/// not an in-process capability. A backend codec can persist these owned
/// fields without serializing a trusted Boolean or private token.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PersistedJournalEventKind {
    QueueAccepted {
        expires_at: UnixMillis,
        policy_generation: u64,
    },
    WorkerEpochAdvanced {
        new_epoch: u64,
    },
    AttemptStarted {
        attempt_id: String,
    },
    AttemptOutcomeRecorded {
        attempt_id: String,
        outcome: AttemptOutcome,
        retry_at: Option<UnixMillis>,
    },
    RetryScheduled {
        not_before: UnixMillis,
    },
    PolicyRevoked {
        policy_generation: u64,
    },
    Expired,
    RecipientReceiptVerified {
        encoded_receipt: Vec<u8>,
        verifier_version: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersistedJournalEvent {
    schema_version: u16,
    stream_revision: u64,
    event_id: String,
    identity: MessageIdentity,
    occurred_at: UnixMillis,
    actor: JournalActor,
    kind: PersistedJournalEventKind,
}

impl PersistedJournalEvent {
    pub const CURRENT_SCHEMA_VERSION: u16 = 1;

    pub fn new(
        stream_revision: u64,
        event_id: impl Into<String>,
        identity: MessageIdentity,
        occurred_at: UnixMillis,
        actor: JournalActor,
        kind: PersistedJournalEventKind,
    ) -> Result<Self, JournalError> {
        Self::new_versioned(
            Self::CURRENT_SCHEMA_VERSION,
            stream_revision,
            event_id,
            identity,
            occurred_at,
            actor,
            kind,
        )
    }

    /// Used by storage decoders. Unsupported versions fail closed so an
    /// unknown schema is never silently interpreted as the current contract.
    pub fn new_versioned(
        schema_version: u16,
        stream_revision: u64,
        event_id: impl Into<String>,
        identity: MessageIdentity,
        occurred_at: UnixMillis,
        actor: JournalActor,
        kind: PersistedJournalEventKind,
    ) -> Result<Self, JournalError> {
        if schema_version != Self::CURRENT_SCHEMA_VERSION {
            return Err(JournalError::UnsupportedEventSchemaVersion);
        }
        if stream_revision == 0 {
            return Err(JournalError::InvalidStreamRevision);
        }
        let event_id = event_id.into();
        if !valid_identifier(&event_id, MAX_EVENT_ID_BYTES) {
            return Err(JournalError::InvalidEventId);
        }
        match &kind {
            PersistedJournalEventKind::AttemptStarted { attempt_id }
            | PersistedJournalEventKind::AttemptOutcomeRecorded { attempt_id, .. } => {
                if !valid_identifier(attempt_id, MAX_ATTEMPT_ID_BYTES) {
                    return Err(JournalError::InvalidAttemptId);
                }
            }
            PersistedJournalEventKind::RecipientReceiptVerified {
                encoded_receipt,
                verifier_version,
            } => {
                if encoded_receipt.is_empty() || encoded_receipt.len() > MAX_RECEIPT_BYTES {
                    return Err(JournalError::InvalidReceiptEvidence);
                }
                if !valid_identifier(verifier_version, 64) {
                    return Err(JournalError::InvalidVerifierVersion);
                }
            }
            _ => {}
        }
        Ok(Self {
            schema_version,
            stream_revision,
            event_id,
            identity,
            occurred_at,
            actor,
            kind,
        })
    }

    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub const fn stream_revision(&self) -> u64 {
        self.stream_revision
    }


    /// Capture a live validated event into a persistence DTO. Receipt bytes
    /// and the verifier version are preserved so replay can re-authenticate.
    pub fn capture(
        event: &JournalEvent,
        stream_revision: u64,
    ) -> Result<Self, JournalError> {
        let kind = match event.kind() {
            JournalEventKind::Queued { expires_at, policy_generation } => {
                PersistedJournalEventKind::QueueAccepted {
                    expires_at: *expires_at,
                    policy_generation: *policy_generation,
                }
            }
            JournalEventKind::WorkerEpochAdvanced { new_epoch } => {
                PersistedJournalEventKind::WorkerEpochAdvanced {
                    new_epoch: *new_epoch,
                }
            }
            JournalEventKind::AttemptStarted { attempt_id } => {
                PersistedJournalEventKind::AttemptStarted {
                    attempt_id: attempt_id.clone(),
                }
            }
            JournalEventKind::AttemptOutcomeRecorded { attempt_id, outcome, retry_at } => {
                PersistedJournalEventKind::AttemptOutcomeRecorded {
                    attempt_id: attempt_id.clone(),
                    outcome: *outcome,
                    retry_at: *retry_at,
                }
            }
            JournalEventKind::RetryScheduled { not_before } => {
                PersistedJournalEventKind::RetryScheduled {
                    not_before: *not_before,
                }
            }
            JournalEventKind::PolicyRevoked { policy_generation } => {
                PersistedJournalEventKind::PolicyRevoked {
                    policy_generation: *policy_generation,
                }
            }
            JournalEventKind::Expired => PersistedJournalEventKind::Expired,
            JournalEventKind::RecipientReceiptVerified { evidence } => {
                PersistedJournalEventKind::RecipientReceiptVerified {
                    encoded_receipt: evidence.encoded_receipt().to_vec(),
                    verifier_version: evidence.verifier_version().to_owned(),
                }
            }
        };
        Self::new(
            stream_revision,
            event.event_id(),
            event.identity().clone(),
            event.occurred_at(),
            event.actor(),
            kind,
        )
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

    pub const fn kind(&self) -> &PersistedJournalEventKind {
        &self.kind
    }

    /// Reverify and append this record at its recorded sequence. Gaps,
    /// reordering, or a forged sequence fail with compare-and-swap conflict.
    /// Exact duplicate replay remains idempotent only at the original revision.
    pub fn replay_into<J: DeliveryJournal, V: TrustedRecipientReceiptVerifier>(
        &self,
        journal: &mut J,
        verifier: &V,
        expected_verifier_version: &str,
    ) -> Result<AppendOutcome, JournalError> {
        let expected_revision = self
            .stream_revision
            .checked_sub(1)
            .ok_or(JournalError::InvalidStreamRevision)?;
        let event = self.reverify(verifier, expected_verifier_version)?;
        match journal.append(expected_revision, event)? {
            AppendOutcome::Appended { revision } if revision == self.stream_revision => {
                Ok(AppendOutcome::Appended { revision })
            }
            AppendOutcome::Duplicate { original_revision }
                if original_revision == self.stream_revision =>
            {
                Ok(AppendOutcome::Duplicate { original_revision })
            }
            _ => Err(JournalError::StreamRevisionMismatch),
        }
    }

    /// Rehydrate after decoding stored bytes. A receipt can be restored only
    /// when the caller selects the verifier registered for the exact stored
    /// version; raw receipt bytes are re-verified for this event's identity
    /// and envelope commitment before a trusted capability is recreated.
    pub fn reverify<V: TrustedRecipientReceiptVerifier>(
        &self,
        verifier: &V,
        expected_verifier_version: &str,
    ) -> Result<JournalEvent, JournalError> {
        if self.schema_version != Self::CURRENT_SCHEMA_VERSION {
            return Err(JournalError::UnsupportedEventSchemaVersion);
        }
        let kind = match &self.kind {
            PersistedJournalEventKind::QueueAccepted { expires_at, policy_generation } => {
                JournalEventKind::Queued {
                    expires_at: *expires_at,
                    policy_generation: *policy_generation,
                }
            }
            PersistedJournalEventKind::WorkerEpochAdvanced { new_epoch } => {
                JournalEventKind::WorkerEpochAdvanced {
                    new_epoch: *new_epoch,
                }
            }
            PersistedJournalEventKind::AttemptStarted { attempt_id } => {
                JournalEventKind::AttemptStarted {
                    attempt_id: attempt_id.clone(),
                }
            }
            PersistedJournalEventKind::AttemptOutcomeRecorded { attempt_id, outcome, retry_at } => {
                JournalEventKind::AttemptOutcomeRecorded {
                    attempt_id: attempt_id.clone(),
                    outcome: *outcome,
                    retry_at: *retry_at,
                }
            }
            PersistedJournalEventKind::RetryScheduled { not_before } => {
                JournalEventKind::RetryScheduled {
                    not_before: *not_before,
                }
            }
            PersistedJournalEventKind::PolicyRevoked { policy_generation } => {
                JournalEventKind::PolicyRevoked {
                    policy_generation: *policy_generation,
                }
            }
            PersistedJournalEventKind::Expired => JournalEventKind::Expired,
            PersistedJournalEventKind::RecipientReceiptVerified {
                encoded_receipt,
                verifier_version,
            } => {
                if expected_verifier_version != verifier_version {
                    return Err(JournalError::ReceiptVerifierVersionMismatch);
                }
                let evidence = VerifiedReceiptEvidence::verify(
                    verifier,
                    encoded_receipt,
                    self.identity.logical_message_id(),
                    self.identity.recipient_id(),
                    *self.identity.envelope_commitment(),
                    verifier_version.clone(),
                )
                .map_err(|_| JournalError::ReceiptVerificationFailed)?;
                JournalEventKind::RecipientReceiptVerified { evidence }
            }
        };
        JournalEvent::new(
            self.event_id.clone(),
            self.identity.clone(),
            self.occurred_at,
            self.actor,
            kind,
        )
        .map_err(|_| JournalError::InvalidPersistedRecord)
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
    InvalidReceiptEvidence,
    InvalidVerifierVersion,
    InvalidPersistedRecord,
    InvalidStreamRevision,
    StreamRevisionMismatch,
    UnsupportedEventSchemaVersion,
    ReceiptVerificationFailed,
    ReceiptVerifierVersionMismatch,
    RevisionConflict { expected: u64, actual: u64 },
    RevisionExhausted,
    EventLimitReached,
    StorageFailure,
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

    /// Return an owned snapshot so a database-backed implementation need not
    /// retain references into a process-local cache.
    fn snapshot(
        &self,
        identity: &MessageIdentity,
    ) -> Result<Option<JournalSnapshot>, JournalError>;

    /// Return an owned, bounded history. Persistent backends must enforce the
    /// event retention limit and surface storage failures instead of pretending
    /// an unavailable read means an empty stream.
    fn history(
        &self,
        identity: &MessageIdentity,
    ) -> Result<Vec<JournalEntry>, JournalError>;
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
        if self
            .streams
            .get(&event.identity)
            .is_some_and(|(_, history)| history.len() >= MAX_EVENTS_PER_MESSAGE)
        {
            return Err(JournalError::EventLimitReached);
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

    fn snapshot(
        &self,
        identity: &MessageIdentity,
    ) -> Result<Option<JournalSnapshot>, JournalError> {
        Ok(self
            .streams
            .get(identity)
            .map(|(snapshot, _)| snapshot.clone()))
    }

    fn history(
        &self,
        identity: &MessageIdentity,
    ) -> Result<Vec<JournalEntry>, JournalError> {
        Ok(self
            .streams
            .get(identity)
            .map(|(_, history)| history.clone())
            .unwrap_or_default())
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
                    if snapshot.revoked {
                        if retry_at.is_some() {
                            return Err(JournalError::InvalidTransition);
                        }
                        snapshot.next_attempt_at = None;
                        snapshot.status = if snapshot.may_have_reached_recipient {
                            DurableDeliveryStatus::OutcomeUnknown
                        } else {
                            DurableDeliveryStatus::Revoked
                        };
                    } else {
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
            if snapshot.active_attempt_id.is_some() {
                // The send may already have crossed the network boundary.
                // Preserve the unresolved attempt so its late adapter outcome
                // remains auditable; revoked=true fences every new attempt.
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
    fn persisted_replay_rejects_out_of_order_stream_revisions() {
        let mut source = InMemoryDeliveryJournal::default();
        let identity = queued(&mut source);
        source.append(1, event(
            "event-epoch",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
        )).unwrap();
        let entries = source.history(&identity).unwrap();
        let first = PersistedJournalEvent::capture(entries[0].event(), entries[0].revision()).unwrap();
        let second = PersistedJournalEvent::capture(entries[1].event(), entries[1].revision()).unwrap();

        let mut replayed = InMemoryDeliveryJournal::default();
        assert_eq!(
            second.replay_into(&mut replayed, &TestVerifier, "pulse-test-v1"),
            Err(JournalError::RevisionConflict { expected: 1, actual: 0 })
        );
        first.replay_into(&mut replayed, &TestVerifier, "pulse-test-v1").unwrap();
        second.replay_into(&mut replayed, &TestVerifier, "pulse-test-v1").unwrap();
        assert_eq!(
            replayed.snapshot(&identity).unwrap().unwrap().revision(),
            2
        );
    }

    #[test]
    fn persisted_event_schema_versions_fail_closed() {
        assert_eq!(
            PersistedJournalEvent::new_versioned(
                99,
                1,
                "event-queue",
                identity(),
                UnixMillis::new(1),
                JournalActor::System,
                PersistedJournalEventKind::QueueAccepted {
                    expires_at: UnixMillis::new(10_000),
                    policy_generation: 1,
                },
            ),
            Err(JournalError::UnsupportedEventSchemaVersion)
        );
        let persisted = PersistedJournalEvent::new(
            1,
            "event-queue",
            identity(),
            UnixMillis::new(1),
            JournalActor::System,
            PersistedJournalEventKind::QueueAccepted {
                expires_at: UnixMillis::new(10_000),
                policy_generation: 1,
            },
        ).unwrap();
        assert_eq!(
            persisted.schema_version(),
            PersistedJournalEvent::CURRENT_SCHEMA_VERSION
        );
    }

    #[test]
    fn persisted_receipt_event_round_trips_only_after_verifier_rechecks_it() {
        let evidence = VerifiedReceiptEvidence::verify(
            &TestVerifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();
        let live = event(
            "event-receipt",
            identity(),
            2,
            JournalActor::System,
            JournalEventKind::RecipientReceiptVerified { evidence },
        );
        let persisted = PersistedJournalEvent::capture(&live, 2).unwrap();
        let restored = persisted
            .reverify(&TestVerifier, "pulse-test-v1")
            .unwrap();
        assert_eq!(restored, live);
    }

    #[test]
    fn persisted_receipt_replay_rejects_wrong_verifier_version() {
        let evidence = VerifiedReceiptEvidence::verify(
            &TestVerifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();
        let live = event(
            "event-receipt",
            identity(),
            2,
            JournalActor::System,
            JournalEventKind::RecipientReceiptVerified { evidence },
        );
        let persisted = PersistedJournalEvent::capture(&live, 2);
        assert_eq!(
            persisted.reverify(&TestVerifier, "pulse-test-v0"),
            Err(JournalError::ReceiptVerifierVersionMismatch)
        );
    }

    #[test]
    fn persisted_receipt_replay_rejects_tampered_bytes_and_identity_commitment() {
        let evidence = VerifiedReceiptEvidence::verify(
            &TestVerifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();
        let live = event(
            "event-receipt",
            identity(),
            2,
            JournalActor::System,
            JournalEventKind::RecipientReceiptVerified { evidence },
        );
        let persisted = PersistedJournalEvent::capture(&live, 2);

        let mut tampered_bytes = persisted.clone();
        if let PersistedJournalEventKind::RecipientReceiptVerified {
            encoded_receipt, ..
        } = &mut tampered_bytes.kind
        {
            *encoded_receipt = b"tampered".to_vec();
        } else {
            unreachable!("live event must be receipt-bearing");
        }
        assert_eq!(
            tampered_bytes.reverify(&TestVerifier, "pulse-test-v1"),
            Err(JournalError::ReceiptVerificationFailed)
        );

        let mut tampered_identity = persisted;
        tampered_identity.identity = MessageIdentity::new(
            "message-1",
            "recipient-1",
            [9; 32],
        ).unwrap();
        assert_eq!(
            tampered_identity.reverify(&TestVerifier, "pulse-test-v1"),
            Err(JournalError::ReceiptVerificationFailed)
        );
    }

    #[test]
    fn replaying_persisted_event_history_rebuilds_identical_projection() {
        let mut original = InMemoryDeliveryJournal::default();
        let identity = queued(&mut original);
        original.append(1, event(
            "event-epoch",
            identity.clone(),
            2,
            JournalActor::System,
            JournalEventKind::WorkerEpochAdvanced { new_epoch: 1 },
        )).unwrap();
        original.append(2, event(
            "event-attempt",
            identity.clone(),
            3,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptStarted { attempt_id: "attempt-1".into() },
        )).unwrap();
        original.append(3, event(
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
        let evidence = VerifiedReceiptEvidence::verify(
            &TestVerifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();
        original.append(4, event(
            "event-receipt",
            identity.clone(),
            5,
            JournalActor::System,
            JournalEventKind::RecipientReceiptVerified { evidence },
        )).unwrap();

        let source_history = original.history(&identity).unwrap();
        let expected = original.snapshot(&identity).unwrap().unwrap();
        let mut replayed = InMemoryDeliveryJournal::default();
        for entry in source_history {
            let persisted = PersistedJournalEvent::capture(entry.event(), entry.revision()).unwrap();
            persisted
                .replay_into(&mut replayed, &TestVerifier, "pulse-test-v1")
                .unwrap();
        }
        assert_eq!(
            replayed.snapshot(&identity).unwrap().unwrap(),
            expected
        );
        assert_eq!(
            replayed.history(&identity).unwrap().len(),
            expected.revision() as usize
        );
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
        assert_eq!(journal.snapshot(&identity).unwrap().unwrap().revision(), 1);
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
        assert_eq!(journal.snapshot(&identity).unwrap().unwrap().revision(), 1);
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
            journal.snapshot(&identity).unwrap().unwrap().status(),
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
            journal.snapshot(&identity).unwrap().unwrap().status(),
            DurableDeliveryStatus::Delivered
        );
        assert_eq!(journal.snapshot(&identity).unwrap().unwrap().revision(), 2);
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
        assert_eq!(journal.snapshot(&identity).unwrap().unwrap().revision(), 4);
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
            journal.snapshot(&identity).unwrap().unwrap().status(),
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
        assert!(journal.snapshot(&identity).unwrap().unwrap().revoked());
        assert_eq!(
            journal.snapshot(&identity).unwrap().unwrap().status(),
            DurableDeliveryStatus::OutcomeUnknown
        );
        assert_eq!(
            journal.snapshot(&identity).unwrap().unwrap().active_attempt_id(),
            Some("attempt-1")
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

        // The send result for the pre-revocation attempt can arrive later.
        // Record it for audit, but revocation still prevents another attempt.
        journal.append(4, event(
            "event-late-ack-after-revoke",
            identity.clone(),
            5,
            JournalActor::Worker { epoch: 1 },
            JournalEventKind::AttemptOutcomeRecorded {
                attempt_id: "attempt-1".into(),
                outcome: AttemptOutcome::GatewayAcknowledged,
                retry_at: None,
            },
        )).unwrap();
        assert!(journal.snapshot(&identity).unwrap().unwrap().revoked());
        assert_eq!(
            journal.snapshot(&identity).unwrap().unwrap().status(),
            DurableDeliveryStatus::AwaitingReceipt
        );

        let evidence = VerifiedReceiptEvidence::verify(
            &TestVerifier,
            b"receipt-ok",
            "message-1",
            "recipient-1",
            ZERO_COMMITMENT,
            "pulse-test-v1",
        ).unwrap();
        journal.append(5, event(
            "event-receipt-after-revoke",
            identity.clone(),
            6,
            JournalActor::System,
            JournalEventKind::RecipientReceiptVerified { evidence },
        )).unwrap();
        assert_eq!(
            journal.snapshot(&identity).unwrap().unwrap().status(),
            DurableDeliveryStatus::Delivered
        );
    }

    #[test]
    fn journal_history_is_bounded_and_overflow_does_not_mutate_state() {
        let mut journal = InMemoryDeliveryJournal::default();
        let identity = queued(&mut journal);
        for epoch in 1..MAX_EVENTS_PER_MESSAGE as u64 {
            let event_id = format!("event-epoch-{epoch}");
            let revision = journal.snapshot(&identity).unwrap().unwrap().revision();
            journal.append(
                revision,
                event(
                    &event_id,
                    identity.clone(),
                    epoch + 1,
                    JournalActor::System,
                    JournalEventKind::WorkerEpochAdvanced { new_epoch: epoch },
                ),
            )
            .unwrap();
        }
        let before = journal.snapshot(&identity).unwrap().unwrap().revision();
        assert_eq!(before as usize, MAX_EVENTS_PER_MESSAGE);
        assert_eq!(
            journal.append(
                before,
                event(
                    "event-over-limit",
                    identity.clone(),
                    MAX_EVENTS_PER_MESSAGE as u64 + 2,
                    JournalActor::System,
                    JournalEventKind::WorkerEpochAdvanced {
                        new_epoch: MAX_EVENTS_PER_MESSAGE as u64,
                    },
                ),
            ),
            Err(JournalError::EventLimitReached)
        );
        assert_eq!(
            journal.history(&identity).unwrap().len(),
            MAX_EVENTS_PER_MESSAGE
        );
        assert_eq!(
            journal.snapshot(&identity).unwrap().unwrap().revision(),
            before
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
            journal.snapshot(&identity).unwrap().unwrap().status(),
            DurableDeliveryStatus::AwaitingReceipt
        );
    }
}

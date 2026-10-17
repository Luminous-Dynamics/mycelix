//! Revision-fenced reservation state machine for one authoritative capacity pool.
//!
//! This is a serial, in-memory reference protocol. It demonstrates atomic state
//! transitions, revision/fencing checks, idempotency and lease expiry; it does
//! not implement locks, durable storage, consensus, distributed transactions or
//! an external inventory adapter. A caller must put each pool behind exactly
//! one authoritative transaction domain before making any operational claim.

use std::collections::{BTreeMap, BTreeSet};

use super::{EvidenceOrigin, EvidenceStatus};

const EVALUATOR_PROFILE: &str = "mycelix:reservation-cas-reference:v1";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CapacityPoolSpec {
    pub pool_id: String,
    pub unit: String,
    pub time_bucket: String,
    pub capacity_quantity: u64,
    pub snapshot_digest: String,
    pub evidence_status: EvidenceStatus,
    pub evidence_origin: EvidenceOrigin,
    pub observed_at: u64,
    pub valid_until: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ReservationState {
    Active,
    Committed,
    Consumed,
    Released,
    Expired,
}

impl ReservationState {
    fn occupies_capacity(self) -> bool {
        matches!(self, Self::Active | Self::Committed | Self::Consumed)
    }

    fn counts_as_reserved(self) -> bool {
        matches!(self, Self::Active | Self::Committed)
    }

    fn counts_as_consumed(self) -> bool {
        self == Self::Consumed
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReserveCommand {
    pub command_id: String,
    pub idempotency_key: String,
    pub reservation_id: String,
    pub request_id: String,
    pub pool_id: String,
    pub expected_revision: u64,
    pub expected_fencing_token: u64,
    pub expected_snapshot_digest: String,
    pub quantity: u64,
    pub issued_at: u64,
    pub expires_at: u64,
    pub policy_digest: String,
    pub input_manifest_digest: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReservationReceipt {
    pub command_id: String,
    pub idempotency_key: String,
    pub reservation_id: String,
    pub request_id: String,
    pub pool_id: String,
    pub unit: String,
    pub time_bucket: String,
    pub quantity: u64,
    pub issued_at: u64,
    pub expires_at: u64,
    pub policy_digest: String,
    pub input_manifest_digest: String,
    pub capacity_snapshot_digest: String,
    pub prior_revision: u64,
    pub revision_after_reserve: u64,
    /// Monotonic per-pool fence issued by the authoritative state machine.
    pub fencing_token: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReservationAction {
    Commit,
    Consume,
    Release,
    Expire,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReservationTransitionCommand {
    pub command_id: String,
    pub reservation_id: String,
    pub expected_revision: u64,
    pub expected_fencing_token: u64,
    pub at: u64,
    pub action: ReservationAction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReservationEventKind {
    Reserved,
    Committed,
    Consumed,
    Released,
    Expired,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReservationEvent {
    pub sequence: u64,
    pub event_id: String,
    pub command_id: String,
    pub kind: ReservationEventKind,
    pub pool_id: String,
    pub reservation_id: String,
    pub quantity: u64,
    pub at: u64,
    pub prior_revision: u64,
    pub new_revision: u64,
    pub fencing_token: u64,
    pub state_after: ReservationState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandDisposition {
    Inserted,
    IdempotentReplay,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReserveResult {
    pub disposition: CommandDisposition,
    /// This is the immutable receipt from reservation creation. Consult
    /// reservation_state() for the current state after later transitions.
    pub receipt: ReservationReceipt,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransitionResult {
    pub disposition: CommandDisposition,
    /// The original event is returned on exact retry; no new event is appended.
    pub event: ReservationEvent,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PoolCapacityReport {
    pub pool_id: String,
    pub unit: String,
    pub time_bucket: String,
    pub capacity_quantity: u64,
    pub reserved_quantity: u64,
    pub consumed_quantity: u64,
    pub remaining_quantity: u64,
    pub revision: u64,
    pub fencing_token: u64,
    pub snapshot_digest: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReservationError {
    InvalidIdentity,
    InvalidPool,
    DuplicatePool,
    UnknownPool,
    DuplicateReservationId,
    UnknownReservation,
    IdempotencyKeyConflict,
    CommandIdentityConflict,
    StaleRevision,
    StaleFencingToken,
    SnapshotMismatch,
    EvidenceNotVerified,
    EvidenceNotLive,
    EvidenceFromFuture,
    EvidenceExpired,
    ZeroQuantity,
    InvalidLease,
    CapacityUnavailable,
    LeaseExpired,
    LeaseNotExpired,
    InvalidTransition,
    NonMonotonicTime,
    ArithmeticOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PoolState {
    spec: CapacityPoolSpec,
    revision: u64,
    fencing_token: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct ReservationRecord {
    receipt: ReservationReceipt,
    state: ReservationState,
    last_transition_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum CommandIdentity {
    Reserve(ReserveCommand),
    Transition(ReservationTransitionCommand),
}

/// One in-memory serial authority for multiple named capacity pools.
/// Every successful mutation increments both revision and fencing token.
/// A compare-and-swap mismatch performs no mutation and appends no event.
#[derive(Default)]
pub struct ReservationBook {
    pools: BTreeMap<String, PoolState>,
    reservations: BTreeMap<String, ReservationRecord>,
    reserve_commands: BTreeMap<String, ReserveCommand>,
    reserve_receipts: BTreeMap<String, ReservationReceipt>,
    used_command_ids: BTreeMap<String, CommandIdentity>,
    command_events: BTreeMap<String, ReservationEvent>,
    events: Vec<ReservationEvent>,
}

impl ReservationBook {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a pre-existing capacity snapshot. Re-registration is rejected:
    /// updating evidence/revisions needs a separately specified refresh command.
    pub fn register_pool(&mut self, spec: CapacityPoolSpec) -> Result<(), ReservationError> {
        if [
            spec.pool_id.as_str(),
            spec.unit.as_str(),
            spec.time_bucket.as_str(),
            spec.snapshot_digest.as_str(),
        ]
        .iter()
        .any(|value| !non_empty(value))
        {
            return Err(ReservationError::InvalidIdentity);
        }
        if spec.capacity_quantity == 0 {
            return Err(ReservationError::InvalidPool);
        }
        if self.pools.contains_key(&spec.pool_id) {
            return Err(ReservationError::DuplicatePool);
        }
        self.pools.insert(
            spec.pool_id.clone(),
            PoolState {
                spec,
                revision: 1,
                fencing_token: 1,
            },
        );
        Ok(())
    }

    pub fn pool_version(&self, pool_id: &str) -> Option<(u64, u64, String)> {
        self.pools.get(pool_id).map(|pool| {
            (
                pool.revision,
                pool.fencing_token,
                pool.spec.snapshot_digest.clone(),
            )
        })
    }

    pub fn events(&self) -> &[ReservationEvent] {
        &self.events
    }

    pub fn reservation_state(&self, reservation_id: &str) -> Option<ReservationState> {
        self.reservations.get(reservation_id).map(|record| record.state)
    }

    pub fn reservation_receipt(&self, reservation_id: &str) -> Option<&ReservationReceipt> {
        self.reservations
            .get(reservation_id)
            .map(|record| &record.receipt)
    }

    pub fn pool_capacity(&self, pool_id: &str) -> Result<PoolCapacityReport, ReservationError> {
        let pool = self
            .pools
            .get(pool_id)
            .ok_or(ReservationError::UnknownPool)?;
        let (reserved, consumed, occupied) = self.occupied_quantities(pool_id)?;
        let remaining = pool
            .spec
            .capacity_quantity
            .checked_sub(occupied)
            .ok_or(ReservationError::ArithmeticOverflow)?;
        Ok(PoolCapacityReport {
            pool_id: pool.spec.pool_id.clone(),
            unit: pool.spec.unit.clone(),
            time_bucket: pool.spec.time_bucket.clone(),
            capacity_quantity: pool.spec.capacity_quantity,
            reserved_quantity: reserved,
            consumed_quantity: consumed,
            remaining_quantity: remaining,
            revision: pool.revision,
            fencing_token: pool.fencing_token,
            snapshot_digest: pool.spec.snapshot_digest.clone(),
        })
    }

    /// Atomically check one frozen snapshot and reserve its quantity.
    ///
    /// Exact retries under the same idempotency key return the original receipt.
    /// Conflicting key reuse, stale revision/token/snapshot, invalid evidence,
    /// expiry, arithmetic overflow and capacity exhaustion all fail closed.
    pub fn reserve(
        &mut self,
        command: ReserveCommand,
    ) -> Result<ReserveResult, ReservationError> {
        if !non_empty(&command.idempotency_key) {
            return Err(ReservationError::InvalidIdentity);
        }
        if let Some(existing) = self.reserve_commands.get(&command.idempotency_key) {
            return if existing == &command {
                let receipt = self
                    .reserve_receipts
                    .get(&command.idempotency_key)
                    .cloned()
                    .ok_or(ReservationError::IdempotencyKeyConflict)?;
                Ok(ReserveResult {
                    disposition: CommandDisposition::IdempotentReplay,
                    receipt,
                })
            } else {
                Err(ReservationError::IdempotencyKeyConflict)
            };
        }
        validate_command_identity(&command.command_id, &command.reservation_id)?;
        if self.used_command_ids.contains_key(&command.command_id) {
            return Err(ReservationError::CommandIdentityConflict);
        }
        if self.reservations.contains_key(&command.reservation_id) {
            return Err(ReservationError::DuplicateReservationId);
        }
        if command.quantity == 0 {
            return Err(ReservationError::ZeroQuantity);
        }
        if command.expires_at <= command.issued_at {
            return Err(ReservationError::InvalidLease);
        }
        if !non_empty(&command.request_id)
            || !non_empty(&command.pool_id)
            || !non_empty(&command.expected_snapshot_digest)
            || !non_empty(&command.policy_digest)
            || !non_empty(&command.input_manifest_digest)
        {
            return Err(ReservationError::InvalidIdentity);
        }

        let pool = self
            .pools
            .get(&command.pool_id)
            .ok_or(ReservationError::UnknownPool)?
            .clone();
        if command.expected_revision != pool.revision {
            return Err(ReservationError::StaleRevision);
        }
        if command.expected_fencing_token != pool.fencing_token {
            return Err(ReservationError::StaleFencingToken);
        }
        if command.expected_snapshot_digest != pool.spec.snapshot_digest {
            return Err(ReservationError::SnapshotMismatch);
        }
        validate_evidence(&pool.spec, command.issued_at)?;

        let (_, _, occupied) = self.occupied_quantities(&command.pool_id)?;
        let after = occupied
            .checked_add(command.quantity)
            .ok_or(ReservationError::ArithmeticOverflow)?;
        if after > pool.spec.capacity_quantity {
            return Err(ReservationError::CapacityUnavailable);
        }
        let next_revision = pool
            .revision
            .checked_add(1)
            .ok_or(ReservationError::ArithmeticOverflow)?;
        let next_fencing_token = pool
            .fencing_token
            .checked_add(1)
            .ok_or(ReservationError::ArithmeticOverflow)?;
        let sequence = u64::try_from(self.events.len())
            .map_err(|_| ReservationError::ArithmeticOverflow)?
            .checked_add(1)
            .ok_or(ReservationError::ArithmeticOverflow)?;

        let receipt = ReservationReceipt {
            command_id: command.command_id.clone(),
            idempotency_key: command.idempotency_key.clone(),
            reservation_id: command.reservation_id.clone(),
            request_id: command.request_id.clone(),
            pool_id: command.pool_id.clone(),
            unit: pool.spec.unit.clone(),
            time_bucket: pool.spec.time_bucket.clone(),
            quantity: command.quantity,
            issued_at: command.issued_at,
            expires_at: command.expires_at,
            policy_digest: command.policy_digest.clone(),
            input_manifest_digest: command.input_manifest_digest.clone(),
            capacity_snapshot_digest: pool.spec.snapshot_digest.clone(),
            prior_revision: pool.revision,
            revision_after_reserve: next_revision,
            fencing_token: next_fencing_token,
        };
        let event = ReservationEvent {
            sequence,
            event_id: command.command_id.clone(),
            command_id: command.command_id.clone(),
            kind: ReservationEventKind::Reserved,
            pool_id: command.pool_id.clone(),
            reservation_id: command.reservation_id.clone(),
            quantity: command.quantity,
            at: command.issued_at,
            prior_revision: pool.revision,
            new_revision: next_revision,
            fencing_token: next_fencing_token,
            state_after: ReservationState::Active,
        };

        // All checks and checked arithmetic have completed. Apply the state change
        // and append its receipt/event only after every failure point above.
        let pool_state = self
            .pools
            .get_mut(&command.pool_id)
            .ok_or(ReservationError::UnknownPool)?;
        pool_state.revision = next_revision;
        pool_state.fencing_token = next_fencing_token;
        self.reservations.insert(
            command.reservation_id.clone(),
            ReservationRecord {
                receipt: receipt.clone(),
                state: ReservationState::Active,
                last_transition_at: command.issued_at,
            },
        );
        self.reserve_commands
            .insert(command.idempotency_key.clone(), command.clone());
        self.reserve_receipts
            .insert(command.idempotency_key.clone(), receipt.clone());
        self.used_command_ids
            .insert(command.command_id.clone(), CommandIdentity::Reserve(command));
        self.command_events
            .insert(event.command_id.clone(), event.clone());
        self.events.push(event);
        Ok(ReserveResult {
            disposition: CommandDisposition::Inserted,
            receipt,
        })
    }

    /// Commit or release an active lease, consume a committed reservation, or
    /// explicitly expire an active lease after its expiry time. All transitions
    /// use the current pool revision and fencing token as compare-and-swap guards.
    pub fn transition(
        &mut self,
        command: ReservationTransitionCommand,
    ) -> Result<TransitionResult, ReservationError> {
        if !non_empty(&command.command_id) || !non_empty(&command.reservation_id) {
            return Err(ReservationError::InvalidIdentity);
        }
        if let Some(existing) = self.used_command_ids.get(&command.command_id) {
            return match existing {
                CommandIdentity::Transition(saved) if saved == &command => {
                    let event = self
                        .command_events
                        .get(&command.command_id)
                        .cloned()
                        .ok_or(ReservationError::CommandIdentityConflict)?;
                    Ok(TransitionResult {
                        disposition: CommandDisposition::IdempotentReplay,
                        event,
                    })
                }
                _ => Err(ReservationError::CommandIdentityConflict),
            };
        }

        let record = self
            .reservations
            .get(&command.reservation_id)
            .cloned()
            .ok_or(ReservationError::UnknownReservation)?;
        let pool = self
            .pools
            .get(&record.receipt.pool_id)
            .cloned()
            .ok_or(ReservationError::UnknownPool)?;

        if command.expected_revision != pool.revision {
            return Err(ReservationError::StaleRevision);
        }
        if command.expected_fencing_token != pool.fencing_token {
            return Err(ReservationError::StaleFencingToken);
        }

        if command.at < record.last_transition_at {
            return Err(ReservationError::NonMonotonicTime);
        }

        let (next_state, kind) = match command.action {
            ReservationAction::Commit => {
                if record.state != ReservationState::Active {
                    return Err(ReservationError::InvalidTransition);
                }
                if command.at >= record.receipt.expires_at {
                    return Err(ReservationError::LeaseExpired);
                }
                (ReservationState::Committed, ReservationEventKind::Committed)
            }
            ReservationAction::Consume => {
                if record.state != ReservationState::Committed {
                    return Err(ReservationError::InvalidTransition);
                }
                (ReservationState::Consumed, ReservationEventKind::Consumed)
            }
            ReservationAction::Release => {
                if record.state != ReservationState::Active {
                    return Err(ReservationError::InvalidTransition);
                }
                if command.at >= record.receipt.expires_at {
                    return Err(ReservationError::LeaseExpired);
                }
                (ReservationState::Released, ReservationEventKind::Released)
            }
            ReservationAction::Expire => {
                if record.state != ReservationState::Active {
                    return Err(ReservationError::InvalidTransition);
                }
                if command.at < record.receipt.expires_at {
                    return Err(ReservationError::LeaseNotExpired);
                }
                (ReservationState::Expired, ReservationEventKind::Expired)
            }
        };

        let next_revision = pool
            .revision
            .checked_add(1)
            .ok_or(ReservationError::ArithmeticOverflow)?;
        let next_fencing_token = pool
            .fencing_token
            .checked_add(1)
            .ok_or(ReservationError::ArithmeticOverflow)?;
        let sequence = u64::try_from(self.events.len())
            .map_err(|_| ReservationError::ArithmeticOverflow)?
            .checked_add(1)
            .ok_or(ReservationError::ArithmeticOverflow)?;

        let event = ReservationEvent {
            sequence,
            event_id: command.command_id.clone(),
            command_id: command.command_id.clone(),
            kind,
            pool_id: record.receipt.pool_id.clone(),
            reservation_id: command.reservation_id.clone(),
            quantity: record.receipt.quantity,
            at: command.at,
            prior_revision: pool.revision,
            new_revision: next_revision,
            fencing_token: next_fencing_token,
            state_after: next_state,
        };

        let pool_state = self
            .pools
            .get_mut(&record.receipt.pool_id)
            .ok_or(ReservationError::UnknownPool)?;
        pool_state.revision = next_revision;
        pool_state.fencing_token = next_fencing_token;
        let updated_record = self
            .reservations
            .get_mut(&command.reservation_id)
            .ok_or(ReservationError::UnknownReservation)?;
        updated_record.state = next_state;
        updated_record.last_transition_at = command.at;
        self.used_command_ids.insert(
            command.command_id.clone(),
            CommandIdentity::Transition(command),
        );
        self.command_events
            .insert(event.command_id.clone(), event.clone());
        self.events.push(event.clone());

        Ok(TransitionResult {
            disposition: CommandDisposition::Inserted,
            event,
        })
    }

    fn occupied_quantities(
        &self,
        pool_id: &str,
    ) -> Result<(u64, u64, u64), ReservationError> {
        let mut reserved = 0u64;
        let mut consumed = 0u64;
        let mut occupied = 0u64;
        for record in self.reservations.values() {
            if record.receipt.pool_id != pool_id {
                continue;
            }
            if record.state.counts_as_reserved() {
                reserved = reserved
                    .checked_add(record.receipt.quantity)
                    .ok_or(ReservationError::ArithmeticOverflow)?;
            }
            if record.state.counts_as_consumed() {
                consumed = consumed
                    .checked_add(record.receipt.quantity)
                    .ok_or(ReservationError::ArithmeticOverflow)?;
            }
            if record.state.occupies_capacity() {
                occupied = occupied
                    .checked_add(record.receipt.quantity)
                    .ok_or(ReservationError::ArithmeticOverflow)?;
            }
        }
        Ok((reserved, consumed, occupied))
    }
}

fn validate_command_identity(
    command_id: &str,
    reservation_id: &str,
) -> Result<(), ReservationError> {
    if non_empty(command_id) && non_empty(reservation_id) {
        Ok(())
    } else {
        Err(ReservationError::InvalidIdentity)
    }
}

fn validate_evidence(
    spec: &CapacityPoolSpec,
    now: u64,
) -> Result<(), ReservationError> {
    if spec.evidence_status != EvidenceStatus::Verified {
        return Err(ReservationError::EvidenceNotVerified);
    }
    if spec.evidence_origin != EvidenceOrigin::LiveDomain {
        return Err(ReservationError::EvidenceNotLive);
    }
    if spec.observed_at > now {
        return Err(ReservationError::EvidenceFromFuture);
    }
    if spec.valid_until < now {
        return Err(ReservationError::EvidenceExpired);
    }
    Ok(())
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[cfg(test)]
mod tests {
    use super::{
        CapacityPoolSpec, CommandDisposition, ReservationAction, ReservationBook,
        ReservationError, ReservationEventKind, ReservationState, ReservationTransitionCommand,
        ReserveCommand,
    };
    use crate::{EvidenceOrigin, EvidenceStatus};

    fn pool_spec(capacity: u64) -> CapacityPoolSpec {
        CapacityPoolSpec {
            pool_id: "supplier-site-week-44".to_owned(),
            unit: "repair-kit".to_owned(),
            time_bucket: "week-2026-44".to_owned(),
            capacity_quantity: capacity,
            snapshot_digest: "sha256-capacity-snapshot-v1".to_owned(),
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            observed_at: 5,
            valid_until: 1_000,
        }
    }

    fn register(book: &mut ReservationBook, capacity: u64) {
        book.register_pool(pool_spec(capacity)).expect("register pool");
    }

    fn reserve_command(
        command_id: &str,
        idempotency_key: &str,
        reservation_id: &str,
        quantity: u64,
        revision: u64,
        fence: u64,
        expires_at: u64,
    ) -> ReserveCommand {
        ReserveCommand {
            command_id: command_id.to_owned(),
            idempotency_key: idempotency_key.to_owned(),
            reservation_id: reservation_id.to_owned(),
            request_id: format!("request-{reservation_id}"),
            pool_id: "supplier-site-week-44".to_owned(),
            expected_revision: revision,
            expected_fencing_token: fence,
            expected_snapshot_digest: "sha256-capacity-snapshot-v1".to_owned(),
            quantity,
            issued_at: 10,
            expires_at,
            policy_digest: "policy-sha256-v1".to_owned(),
            input_manifest_digest: "manifest-sha256-v1".to_owned(),
        }
    }

    fn transition_command(
        command_id: &str,
        reservation_id: &str,
        action: ReservationAction,
        at: u64,
        revision: u64,
        fence: u64,
    ) -> ReservationTransitionCommand {
        ReservationTransitionCommand {
            command_id: command_id.to_owned(),
            reservation_id: reservation_id.to_owned(),
            expected_revision: revision,
            expected_fencing_token: fence,
            at,
            action,
        }
    }

    #[test]
    fn only_one_of_one_hundred_attempts_using_same_revision_can_reserve_last_unit() {
        let mut book = ReservationBook::new();
        register(&mut book, 1);
        let mut successes = 0;
        for index in 0..100 {
            let command = reserve_command(
                &format!("cmd-{index}"),
                &format!("idem-{index}"),
                &format!("reservation-{index}"),
                1,
                1,
                1,
                50,
            );
            if book.reserve(command).is_ok() {
                successes += 1;
            }
        }
        assert_eq!(successes, 1);
        let pool = book.pool_capacity("supplier-site-week-44").unwrap();
        assert_eq!(pool.reserved_quantity, 1);
        assert_eq!(pool.consumed_quantity, 0);
        assert_eq!(pool.remaining_quantity, 0);
        assert_eq!(book.events().len(), 1);
    }

    #[test]
    fn exact_reservation_retry_returns_original_receipt_without_double_reserving() {
        let mut book = ReservationBook::new();
        register(&mut book, 4);
        let command = reserve_command("cmd-1", "idem-1", "reservation-1", 2, 1, 1, 50);
        let first = book.reserve(command.clone()).unwrap();
        let retry = book.reserve(command).unwrap();
        assert_eq!(first.disposition, CommandDisposition::Inserted);
        assert_eq!(retry.disposition, CommandDisposition::IdempotentReplay);
        assert_eq!(first.receipt, retry.receipt);
        assert_eq!(book.pool_capacity("supplier-site-week-44").unwrap().reserved_quantity, 2);
        assert_eq!(book.events().len(), 1);
    }

    #[test]
    fn conflicting_idempotency_key_reuse_fails_closed() {
        let mut book = ReservationBook::new();
        register(&mut book, 4);
        book.reserve(reserve_command("cmd-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap();
        let conflict = reserve_command("cmd-1", "idem-1", "reservation-1", 2, 1, 1, 50);
        assert_eq!(book.reserve(conflict), Err(ReservationError::IdempotencyKeyConflict));
        assert_eq!(book.events().len(), 1);
    }

    #[test]
    fn stale_revision_and_stale_fencing_token_do_not_mutate_state() {
        let mut book = ReservationBook::new();
        register(&mut book, 4);
        book.reserve(reserve_command("cmd-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap();
        let before = book.events().to_vec();
        assert_eq!(
            book.reserve(reserve_command("cmd-2", "idem-2", "reservation-2", 1, 1, 1, 50)),
            Err(ReservationError::StaleRevision)
        );
        assert_eq!(book.events(), before.as_slice());
        assert_eq!(
            book.reserve(reserve_command("cmd-3", "idem-3", "reservation-3", 1, 2, 1, 50)),
            Err(ReservationError::StaleFencingToken)
        );
        assert_eq!(book.events(), before.as_slice());
    }

    #[test]
    fn snapshot_digest_must_match_expected_revision() {
        let mut book = ReservationBook::new();
        register(&mut book, 4);
        let mut command = reserve_command("cmd-1", "idem-1", "reservation-1", 1, 1, 1, 50);
        command.expected_snapshot_digest = "different-snapshot".to_owned();
        assert_eq!(book.reserve(command), Err(ReservationError::SnapshotMismatch));
        assert!(book.events().is_empty());
    }

    #[test]
    fn stale_or_synthetic_pool_evidence_cannot_reserve_capacity() {
        let mut stale = ReservationBook::new();
        let mut spec = pool_spec(4);
        spec.valid_until = 9;
        stale.register_pool(spec).unwrap();
        assert_eq!(
            stale.reserve(reserve_command("cmd-1", "idem-1", "reservation-1", 1, 1, 1, 50)),
            Err(ReservationError::EvidenceExpired)
        );
        assert!(stale.events().is_empty());

        let mut synthetic = ReservationBook::new();
        let mut spec = pool_spec(4);
        spec.evidence_origin = EvidenceOrigin::SyntheticSimulation;
        synthetic.register_pool(spec).unwrap();
        assert_eq!(
            synthetic.reserve(reserve_command("cmd-1", "idem-1", "reservation-1", 1, 1, 1, 50)),
            Err(ReservationError::EvidenceNotLive)
        );
        assert!(synthetic.events().is_empty());
    }

    #[test]
    fn capacity_released_by_expiry_requires_explicit_expire_transition() {
        let mut book = ReservationBook::new();
        register(&mut book, 1);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 20)).unwrap().receipt;
        assert_eq!(book.pool_capacity(&receipt.pool_id).unwrap().remaining_quantity, 0);
        let version = book.pool_version(&receipt.pool_id).unwrap();
        assert_eq!(
            book.transition(transition_command("expire-early", "reservation-1", ReservationAction::Expire, 19, version.0, version.1)),
            Err(ReservationError::LeaseNotExpired)
        );
        assert_eq!(book.reservation_state("reservation-1"), Some(ReservationState::Active));
        let version = book.pool_version(&receipt.pool_id).unwrap();
        let expired = book.transition(transition_command("expire-1", "reservation-1", ReservationAction::Expire, 20, version.0, version.1)).unwrap();
        assert_eq!(expired.event.kind, ReservationEventKind::Expired);
        assert_eq!(book.reservation_state("reservation-1"), Some(ReservationState::Expired));
        assert_eq!(book.pool_capacity(&receipt.pool_id).unwrap().remaining_quantity, 1);
    }

    #[test]
    fn committed_reservation_cannot_be_released_and_keeps_capacity_occupied() {
        let mut book = ReservationBook::new();
        register(&mut book, 2);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap().receipt;
        let version = book.pool_version(&receipt.pool_id).unwrap();
        book.transition(transition_command("commit-1", "reservation-1", ReservationAction::Commit, 15, version.0, version.1)).unwrap();
        let version = book.pool_version(&receipt.pool_id).unwrap();
        assert_eq!(
            book.transition(transition_command("release-committed", "reservation-1", ReservationAction::Release, 16, version.0, version.1)),
            Err(ReservationError::InvalidTransition)
        );
        assert_eq!(book.reservation_state("reservation-1"), Some(ReservationState::Committed));
        assert_eq!(book.pool_capacity(&receipt.pool_id).unwrap().remaining_quantity, 1);
    }

    #[test]
    fn consume_requires_commit_and_keeps_capacity_counted_as_consumed() {
        let mut book = ReservationBook::new();
        register(&mut book, 2);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap().receipt;
        let version = book.pool_version(&receipt.pool_id).unwrap();
        assert_eq!(
            book.transition(transition_command("consume-early", "reservation-1", ReservationAction::Consume, 15, version.0, version.1)),
            Err(ReservationError::InvalidTransition)
        );
        let version = book.pool_version(&receipt.pool_id).unwrap();
        book.transition(transition_command("commit-1", "reservation-1", ReservationAction::Commit, 15, version.0, version.1)).unwrap();
        let version = book.pool_version(&receipt.pool_id).unwrap();
        book.transition(transition_command("consume-1", "reservation-1", ReservationAction::Consume, 16, version.0, version.1)).unwrap();
        let pool = book.pool_capacity(&receipt.pool_id).unwrap();
        assert_eq!(pool.reserved_quantity, 0);
        assert_eq!(pool.consumed_quantity, 1);
        assert_eq!(pool.remaining_quantity, 1);
        assert_eq!(book.reservation_state("reservation-1"), Some(ReservationState::Consumed));
    }

    #[test]
    fn exact_transition_retry_does_not_append_a_second_event() {
        let mut book = ReservationBook::new();
        register(&mut book, 2);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap().receipt;
        let version = book.pool_version(&receipt.pool_id).unwrap();
        let command = transition_command("commit-1", "reservation-1", ReservationAction::Commit, 15, version.0, version.1);
        let first = book.transition(command.clone()).unwrap();
        let second = book.transition(command).unwrap();
        assert_eq!(first.disposition, CommandDisposition::Inserted);
        assert_eq!(second.disposition, CommandDisposition::IdempotentReplay);
        assert_eq!(first.event, second.event);
        assert_eq!(book.events().len(), 2);
    }

    #[test]
    fn commit_at_or_after_expiry_fails_without_implicitly_mutating_state() {
        let mut book = ReservationBook::new();
        register(&mut book, 1);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 20)).unwrap().receipt;
        let version = book.pool_version(&receipt.pool_id).unwrap();
        let before = book.events().to_vec();
        assert_eq!(
            book.transition(transition_command("commit-expired", "reservation-1", ReservationAction::Commit, 20, version.0, version.1)),
            Err(ReservationError::LeaseExpired)
        );
        assert_eq!(book.reservation_state("reservation-1"), Some(ReservationState::Active));
        assert_eq!(book.events(), before.as_slice());
    }

    #[test]
    fn released_capacity_can_be_reserved_again_only_under_new_revision() {
        let mut book = ReservationBook::new();
        register(&mut book, 1);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap().receipt;
        let version = book.pool_version(&receipt.pool_id).unwrap();
        book.transition(transition_command("release-1", "reservation-1", ReservationAction::Release, 15, version.0, version.1)).unwrap();
        let version = book.pool_version(&receipt.pool_id).unwrap();
        let next = reserve_command("reserve-2", "idem-2", "reservation-2", 1, version.0, version.1, 50);
        let second = book.reserve(next).unwrap();
        assert_eq!(second.receipt.reservation_id, "reservation-2");
        assert_eq!(book.pool_capacity(&receipt.pool_id).unwrap().remaining_quantity, 0);
    }

    #[test]
    fn duplicate_pool_and_reservation_identities_are_not_overwritten() {
        let mut book = ReservationBook::new();
        register(&mut book, 2);
        assert_eq!(book.register_pool(pool_spec(9)), Err(ReservationError::DuplicatePool));
        book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap();
        let version = book.pool_version("supplier-site-week-44").unwrap();
        assert_eq!(
            book.reserve(reserve_command("reserve-2", "idem-2", "reservation-1", 1, version.0, version.1, 50)),
            Err(ReservationError::DuplicateReservationId)
        );
        assert_eq!(book.pool_capacity("supplier-site-week-44").unwrap().reserved_quantity, 1);
    }

    #[test]
    fn operation_command_identity_cannot_be_reused_for_a_different_action() {
        let mut book = ReservationBook::new();
        register(&mut book, 2);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap().receipt;
        let version = book.pool_version(&receipt.pool_id).unwrap();
        book.transition(transition_command("op-1", "reservation-1", ReservationAction::Commit, 15, version.0, version.1)).unwrap();
        let version = book.pool_version(&receipt.pool_id).unwrap();
        assert_eq!(
            book.transition(transition_command("op-1", "reservation-1", ReservationAction::Consume, 16, version.0, version.1)),
            Err(ReservationError::CommandIdentityConflict)
        );
    }

    #[test]
    fn registered_pool_snapshot_expiring_before_reservation_is_rejected() {
        let mut book = ReservationBook::new();
        let mut spec = pool_spec(3);
        spec.valid_until = 9;
        book.register_pool(spec).unwrap();
        assert_eq!(
            book.reserve(reserve_command("cmd-1", "idem-1", "reservation-1", 1, 1, 1, 50)),
            Err(ReservationError::EvidenceExpired)
        );
        assert!(book.events().is_empty());
    }

    #[test]
    fn transition_time_cannot_move_backwards_from_reservation_creation() {
        let mut book = ReservationBook::new();
        register(&mut book, 2);
        let receipt = book.reserve(reserve_command("reserve-1", "idem-1", "reservation-1", 1, 1, 1, 50)).unwrap().receipt;
        let version = book.pool_version(&receipt.pool_id).unwrap();
        assert_eq!(
            book.transition(transition_command("commit-before-issued", "reservation-1", ReservationAction::Commit, 9, version.0, version.1)),
            Err(ReservationError::NonMonotonicTime)
        );
        assert_eq!(book.reservation_state("reservation-1"), Some(ReservationState::Active));
        assert_eq!(book.events().len(), 1);
    }
}

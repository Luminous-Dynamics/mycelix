//! Small sequential state-machine model for retiring and transferring one escrowed
//! risk-right allocation. This models transition ordering and accounting only; it does
//! not implement signatures, a quorum, a linearizable store, or Holochain validation.

use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Phase {
    Active,
    DrainRequested,
    Reconciled,
    Fenced,
    Transferred,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Apply {
    Applied,
    Duplicate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Error {
    InvalidIdentity,
    InvalidAmount,
    WrongOwnerOrEpoch,
    NewExposureClosed,
    CapacityExceeded,
    NothingOutstanding,
    SettlementTooLarge,
    InvalidTransition,
    IncompleteReconciliation,
    TerminalSequenceMismatch,
    OutstandingMismatch,
    ReplayMismatch,
    ReconciliationConflict,
    Overflow,
}

#[derive(Clone, Debug, Eq, PartialEq)]
enum Event {
    Obligation { owner: String, epoch: u64, amount: u64 },
    Settlement { owner: String, epoch: u64, amount: u64 },
    SuccessorObligation { owner: String, epoch: u64, amount: u64 },
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Reconciliation {
    terminal_sequence: u64,
    outstanding: u64,
    pending_exposure: u64,
    transferable_capacity: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Successor {
    receipt_id: String,
    owner: String,
    epoch: u64,
    capacity: u64,
    outstanding: u64,
}

/// One allocation's retirement lifecycle. The original owner remains responsible
/// for its already-incurred outstanding exposure after the unused slice is transferred.
#[derive(Default)]
pub struct FenceBook {
    owner: String,
    epoch: u64,
    capacity: u64,
    outstanding: u64,
    sequence: u64,
    phase: Option<Phase>,
    drain_id: Option<String>,
    reconciliation: Option<Reconciliation>,
    fence_id: Option<String>,
    successor: Option<Successor>,
    events: BTreeMap<String, Event>,
}

impl FenceBook {
    pub fn new(owner: &str, epoch: u64, capacity: u64) -> Result<Self, Error> {
        if owner.trim().is_empty() || epoch == 0 {
            return Err(Error::InvalidIdentity);
        }
        if capacity == 0 {
            return Err(Error::InvalidAmount);
        }
        Ok(Self {
            owner: owner.into(),
            epoch,
            capacity,
            outstanding: 0,
            sequence: 0,
            phase: Some(Phase::Active),
            drain_id: None,
            reconciliation: None,
            fence_id: None,
            successor: None,
            events: BTreeMap::new(),
        })
    }

    pub fn phase(&self) -> Phase {
        self.phase.unwrap_or(Phase::Active)
    }

    pub fn owner(&self) -> &str {
        &self.owner
    }

    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    pub fn capacity(&self) -> u64 {
        self.capacity
    }

    pub fn outstanding(&self) -> u64 {
        self.outstanding
    }

    pub fn operation_sequence(&self) -> u64 {
        self.sequence
    }

    pub fn transferable_capacity(&self) -> Option<u64> {
        self.reconciliation.as_ref().map(|value| value.transferable_capacity)
    }

    pub fn successor_outstanding(&self) -> u64 {
        self.successor.as_ref().map_or(0, |value| value.outstanding)
    }

    pub fn successor_capacity(&self) -> Option<u64> {
        self.successor.as_ref().map(|value| value.capacity)
    }

    fn next_sequence(&self) -> Result<u64, Error> {
        self.sequence.checked_add(1).ok_or(Error::Overflow)
    }

    fn replay(&self, event_id: &str, event: &Event) -> Result<Option<Apply>, Error> {
        if event_id.trim().is_empty() {
            return Err(Error::InvalidIdentity);
        }
        match self.events.get(event_id) {
            Some(previous) if previous == event => Ok(Some(Apply::Duplicate)),
            Some(_) => Err(Error::ReplayMismatch),
            None => Ok(None),
        }
    }

    /// Record new exposure only while the current owner/epoch is ACTIVE.
    /// The event ID is consumed only after every check succeeds.
    pub fn record_obligation(
        &mut self,
        event_id: &str,
        owner: &str,
        epoch: u64,
        amount: u64,
    ) -> Result<Apply, Error> {
        let event = Event::Obligation { owner: owner.into(), epoch, amount };
        if let Some(result) = self.replay(event_id, &event)? {
            return Ok(result);
        }
        if amount == 0 {
            return Err(Error::InvalidAmount);
        }
        if owner != self.owner || epoch != self.epoch {
            return Err(Error::WrongOwnerOrEpoch);
        }
        if self.phase() != Phase::Active {
            return Err(Error::NewExposureClosed);
        }

        let next_outstanding = self.outstanding.checked_add(amount).ok_or(Error::Overflow)?;
        if next_outstanding > self.capacity {
            return Err(Error::CapacityExceeded);
        }
        let next_sequence = self.next_sequence()?;
        self.outstanding = next_outstanding;
        self.sequence = next_sequence;
        self.events.insert(event_id.into(), event);
        Ok(Apply::Applied)
    }

    /// Pause new exposure while retaining the ability to settle existing liabilities.
    pub fn request_drain(&mut self, drain_id: &str) -> Result<Apply, Error> {
        if drain_id.trim().is_empty() {
            return Err(Error::InvalidIdentity);
        }
        if let Some(previous) = &self.drain_id {
            return if previous == drain_id {
                Ok(Apply::Duplicate)
            } else {
                Err(Error::ReplayMismatch)
            };
        }
        if self.phase() != Phase::Active {
            return Err(Error::InvalidTransition);
        }
        self.drain_id = Some(drain_id.into());
        self.phase = Some(Phase::DrainRequested);
        Ok(Apply::Applied)
    }

    /// Settle an already-recorded liability owned by the old epoch.
    /// Settlement never reopens new exposure, even if it frees outstanding capacity.
    pub fn settle(
        &mut self,
        event_id: &str,
        owner: &str,
        epoch: u64,
        amount: u64,
    ) -> Result<Apply, Error> {
        let event = Event::Settlement { owner: owner.into(), epoch, amount };
        if let Some(result) = self.replay(event_id, &event)? {
            return Ok(result);
        }
        if amount == 0 {
            return Err(Error::InvalidAmount);
        }
        if owner != self.owner || epoch != self.epoch {
            return Err(Error::WrongOwnerOrEpoch);
        }
        if amount > self.outstanding {
            if self.outstanding == 0 {
                return Err(Error::NothingOutstanding);
            }
            return Err(Error::SettlementTooLarge);
        }

        let next_outstanding = self.outstanding.checked_sub(amount).ok_or(Error::Overflow)?;
        let next_sequence = self.next_sequence()?;
        self.outstanding = next_outstanding;
        self.sequence = next_sequence;
        self.events.insert(event_id.into(), event);
        Ok(Apply::Applied)
    }

    /// Freeze the operation set at an explicit sequence cut. The boolean is a model
    /// input representing independently verified completeness; this method does not
    /// prove cryptographic or distributed completeness itself.
    pub fn reconcile(
        &mut self,
        terminal_sequence: u64,
        observed_outstanding: u64,
        pending_exposure: u64,
        operation_set_complete: bool,
    ) -> Result<Apply, Error> {
        if !operation_set_complete {
            return Err(Error::IncompleteReconciliation);
        }
        let committed = self.outstanding.checked_add(pending_exposure).ok_or(Error::Overflow)?;
        if committed > self.capacity {
            return Err(Error::CapacityExceeded);
        }
        let candidate = Reconciliation {
            terminal_sequence,
            outstanding: observed_outstanding,
            pending_exposure,
            transferable_capacity: self.capacity - committed,
        };

        if let Some(previous) = &self.reconciliation {
            return if previous == &candidate {
                Ok(Apply::Duplicate)
            } else {
                Err(Error::ReconciliationConflict)
            };
        }
        if self.phase() != Phase::DrainRequested {
            return Err(Error::InvalidTransition);
        }
        if terminal_sequence != self.sequence {
            return Err(Error::TerminalSequenceMismatch);
        }
        if observed_outstanding != self.outstanding {
            return Err(Error::OutstandingMismatch);
        }

        self.reconciliation = Some(candidate);
        self.phase = Some(Phase::Reconciled);
        Ok(Apply::Applied)
    }

    /// Fence the old epoch only after a complete reconciliation at the exact sequence.
    /// A timeout or elapsed wall-clock duration cannot call this transition by itself.
    pub fn fence(&mut self, fence_id: &str, terminal_sequence: u64) -> Result<Apply, Error> {
        if fence_id.trim().is_empty() {
            return Err(Error::InvalidIdentity);
        }
        if let Some(previous) = &self.fence_id {
            return if previous == fence_id {
                Ok(Apply::Duplicate)
            } else {
                Err(Error::ReplayMismatch)
            };
        }
        if self.phase() != Phase::Reconciled {
            return Err(Error::InvalidTransition);
        }
        let reconciliation = self.reconciliation.as_ref().ok_or(Error::InvalidTransition)?;
        if reconciliation.terminal_sequence != terminal_sequence {
            return Err(Error::TerminalSequenceMismatch);
        }

        self.fence_id = Some(fence_id.into());
        self.phase = Some(Phase::Fenced);
        Ok(Apply::Applied)
    }

    /// Transfer no more than the reconciled free risk slice to a newer epoch.
    /// This reference model assumes the caller's fence receipt is valid; it does not
    /// authenticate signers or implement a distributed control authority.
    pub fn transfer(
        &mut self,
        receipt_id: &str,
        new_owner: &str,
        new_epoch: u64,
        requested_capacity: u64,
    ) -> Result<Apply, Error> {
        if receipt_id.trim().is_empty() || new_owner.trim().is_empty() {
            return Err(Error::InvalidIdentity);
        }
        if requested_capacity == 0 {
            return Err(Error::InvalidAmount);
        }
        let candidate = Successor {
            receipt_id: receipt_id.into(),
            owner: new_owner.into(),
            epoch: new_epoch,
            capacity: requested_capacity,
            outstanding: 0,
        };
        if let Some(previous) = &self.successor {
            let same_transfer = previous.receipt_id == candidate.receipt_id
                && previous.owner == candidate.owner
                && previous.epoch == candidate.epoch
                && previous.capacity == candidate.capacity;
            return if same_transfer {
                Ok(Apply::Duplicate)
            } else {
                Err(Error::ReplayMismatch)
            };
        }
        if self.phase() != Phase::Fenced {
            return Err(Error::InvalidTransition);
        }
        if new_owner == self.owner || new_epoch <= self.epoch {
            return Err(Error::WrongOwnerOrEpoch);
        }
        let maximum = self.transferable_capacity().ok_or(Error::InvalidTransition)?;
        if requested_capacity > maximum {
            return Err(Error::CapacityExceeded);
        }

        self.successor = Some(candidate);
        self.phase = Some(Phase::Transferred);
        Ok(Apply::Applied)
    }

    /// Exercise the successor slice only after a committed transfer receipt exists.
    pub fn record_successor_obligation(
        &mut self,
        event_id: &str,
        owner: &str,
        epoch: u64,
        amount: u64,
    ) -> Result<Apply, Error> {
        let event = Event::SuccessorObligation { owner: owner.into(), epoch, amount };
        if let Some(result) = self.replay(event_id, &event)? {
            return Ok(result);
        }
        if amount == 0 {
            return Err(Error::InvalidAmount);
        }
        if self.phase() != Phase::Transferred {
            return Err(Error::InvalidTransition);
        }
        let successor = self.successor.as_ref().ok_or(Error::InvalidTransition)?;
        if owner != successor.owner || epoch != successor.epoch {
            return Err(Error::WrongOwnerOrEpoch);
        }

        let next_successor_outstanding = successor.outstanding.checked_add(amount).ok_or(Error::Overflow)?;
        if next_successor_outstanding > successor.capacity {
            return Err(Error::CapacityExceeded);
        }
        let total = self.outstanding
            .checked_add(self.reconciliation.as_ref().ok_or(Error::InvalidTransition)?.pending_exposure)
            .and_then(|value| value.checked_add(next_successor_outstanding))
            .ok_or(Error::Overflow)?;
        if total > self.capacity {
            return Err(Error::CapacityExceeded);
        }
        let next_sequence = self.next_sequence()?;
        self.successor.as_mut().ok_or(Error::InvalidTransition)?.outstanding = next_successor_outstanding;
        self.sequence = next_sequence;
        self.events.insert(event_id.into(), event);
        Ok(Apply::Applied)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready_book(capacity: u64, amount: u64) -> FenceBook {
        let mut book = FenceBook::new("owner-a", 1, capacity).unwrap();
        book.record_obligation("old:1", "owner-a", 1, amount).unwrap();
        book.request_drain("drain:1").unwrap();
        book
    }

    #[test]
    fn drain_blocks_new_obligations_but_allows_existing_settlement() {
        let mut book = ready_book(100, 40);
        assert_eq!(
            book.record_obligation("old:2", "owner-a", 1, 1),
            Err(Error::NewExposureClosed)
        );
        assert_eq!(book.settle("settle:1", "owner-a", 1, 10), Ok(Apply::Applied));
        assert_eq!(book.outstanding(), 30);
        assert_eq!(book.record_obligation("old:3", "owner-a", 1, 1), Err(Error::NewExposureClosed));
    }

    #[test]
    fn incomplete_or_stale_reconciliation_cannot_fence() {
        let mut book = ready_book(100, 30);
        assert_eq!(
            book.reconcile(book.operation_sequence(), 30, 10, false),
            Err(Error::IncompleteReconciliation)
        );
        assert_eq!(book.phase(), Phase::DrainRequested);
        assert_eq!(book.fence("fence:1", book.operation_sequence()), Err(Error::InvalidTransition));
        assert_eq!(
            book.reconcile(book.operation_sequence().saturating_sub(1), 30, 10, true),
            Err(Error::TerminalSequenceMismatch)
        );
        assert_eq!(book.phase(), Phase::DrainRequested);
    }

    #[test]
    fn transfer_moves_only_reconciled_free_capacity_and_preserves_total_bound() {
        let mut book = ready_book(100, 30);
        let cut = book.operation_sequence();
        assert_eq!(book.reconcile(cut, 30, 10, true), Ok(Apply::Applied));
        assert_eq!(book.transferable_capacity(), Some(60));
        assert_eq!(book.fence("fence:1", cut), Ok(Apply::Applied));
        assert_eq!(book.transfer("transfer:1", "owner-b", 2, 61), Err(Error::CapacityExceeded));
        assert_eq!(book.transfer("transfer:1", "owner-b", 2, 60), Ok(Apply::Applied));
        assert_eq!(book.phase(), Phase::Transferred);
        assert_eq!(book.record_successor_obligation("new:1", "owner-b", 2, 60), Ok(Apply::Applied));
        assert_eq!(book.outstanding() + 10 + book.successor_outstanding(), 100);
        assert_eq!(book.record_successor_obligation("new:2", "owner-b", 2, 1), Err(Error::CapacityExceeded));
    }

    #[test]
    fn old_epoch_cannot_create_exposure_after_drain_or_fence() {
        let mut book = ready_book(100, 25);
        let cut = book.operation_sequence();
        book.reconcile(cut, 25, 0, true).unwrap();
        book.fence("fence:1", cut).unwrap();
        book.transfer("transfer:1", "owner-b", 2, 75).unwrap();
        assert_eq!(
            book.record_obligation("old:late", "owner-a", 1, 1),
            Err(Error::NewExposureClosed)
        );
        assert_eq!(
            book.record_obligation("new:wrong-epoch", "owner-b", 1, 1),
            Err(Error::WrongOwnerOrEpoch)
        );
        assert_eq!(
            book.record_successor_obligation("new:early", "owner-b", 2, 1),
            Ok(Apply::Applied)
        );
    }

    #[test]
    fn identical_drain_fence_and_transfer_retries_are_idempotent() {
        let mut book = ready_book(100, 20);
        assert_eq!(book.request_drain("drain:1"), Ok(Apply::Duplicate));
        let cut = book.operation_sequence();
        assert_eq!(book.reconcile(cut, 20, 0, true), Ok(Apply::Applied));
        assert_eq!(book.reconcile(cut, 20, 0, true), Ok(Apply::Duplicate));
        assert_eq!(book.fence("fence:1", cut), Ok(Apply::Applied));
        assert_eq!(book.fence("fence:1", cut), Ok(Apply::Duplicate));
        assert_eq!(book.transfer("transfer:1", "owner-b", 2, 80), Ok(Apply::Applied));
        assert_eq!(book.transfer("transfer:1", "owner-b", 2, 80), Ok(Apply::Duplicate));
        assert_eq!(book.transfer("transfer:1", "owner-c", 2, 80), Err(Error::ReplayMismatch));
    }

    #[test]
    fn event_identity_cannot_be_reused_across_old_and_successor_epochs() {
        let mut book = ready_book(20, 5);
        let cut = book.operation_sequence();
        book.reconcile(cut, 5, 0, true).unwrap();
        book.fence("fence:1", cut).unwrap();
        book.transfer("transfer:1", "owner-b", 2, 15).unwrap();
        assert_eq!(
            book.record_successor_obligation("old:1", "owner-b", 2, 1),
            Err(Error::ReplayMismatch)
        );
    }

    #[test]
    fn sequential_last_unit_race_accepts_one_claim_and_rejects_the_rest() {
        let mut book = FenceBook::new("owner-a", 1, 1).unwrap();
        let mut applied = 0;
        let mut rejected = 0;
        for index in 0..100 {
            match book.record_obligation(&format!("claim:{index}"), "owner-a", 1, 1) {
                Ok(Apply::Applied) => applied += 1,
                Err(Error::CapacityExceeded) => rejected += 1,
                other => panic!("unexpected result: {other:?}"),
            }
        }
        assert_eq!(applied, 1);
        assert_eq!(rejected, 99);
        assert_eq!(book.outstanding(), 1);
    }

    #[test]
    fn failed_capacity_attempt_does_not_consume_event_identity() {
        let mut book = FenceBook::new("owner-a", 1, 5).unwrap();
        book.record_obligation("tx:1", "owner-a", 1, 5).unwrap();
        assert_eq!(book.record_obligation("tx:retry", "owner-a", 1, 1), Err(Error::CapacityExceeded));
        book.settle("settle:1", "owner-a", 1, 1).unwrap();
        assert_eq!(book.record_obligation("tx:retry", "owner-a", 1, 1), Ok(Apply::Applied));
        assert_eq!(book.outstanding(), 5);
    }
}

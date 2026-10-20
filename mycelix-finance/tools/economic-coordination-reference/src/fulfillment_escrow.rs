//! Experimental escrow-rights model for partition-tolerant partial fulfilment.
//!
//! Each replica receives a disjoint share of the request's finite quantity budget.
//! A replica can reserve only its own unspent rights, so concurrent local decisions
//! cannot exceed the sum of the preallocated shares. This single-process model
//! demonstrates the accounting contract; it does NOT implement authenticated,
//! single-spend transfer certificates or distributed consensus.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EscrowError {
    InvalidRequest,
    InvalidReplica,
    DuplicateReplica,
    AllocationMismatch,
    UnknownReplica,
    InsufficientRights,
    DuplicateClaim,
    MissingClaim,
    ClaimAlreadyFinal,
    InvalidAcceptedQuantity,
    DuplicateTransfer,
    InvalidTransfer,
    ArithmeticOverflow,
    InvariantViolation,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReplicaBalance {
    pub available: u64,
    pub reserved: u64,
    pub accepted: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReservationState {
    Pending,
    Finalized { accepted_quantity: u64 },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EscrowReservation {
    pub claim_id: String,
    pub replica_id: String,
    pub claimed_quantity: u64,
    pub state: ReservationState,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EscrowJournalEntry {
    RightsTransferred {
        transfer_id: String,
        from_replica: String,
        to_replica: String,
        quantity: u64,
    },
    ReservationOpened {
        claim_id: String,
        replica_id: String,
        quantity: u64,
    },
    ReservationFinalized {
        claim_id: String,
        replica_id: String,
        claimed_quantity: u64,
        accepted_quantity: u64,
        released_quantity: u64,
    },
}

/// One-request escrow ledger with disjoint per-replica quantity rights.
///
/// Invariant: sum(available + reserved + accepted) == requested_quantity.
/// Pending claims reserve their full quantity. Finalization permanently consumes
/// only the accepted quantity and returns the rejected remainder to local rights.
/// A transfer moves only currently available rights and never creates rights.
#[derive(Clone, Debug)]
pub struct EscrowReservationLedger {
    request_id: String,
    requested_quantity: u64,
    replicas: BTreeMap<String, ReplicaBalance>,
    reservations: BTreeMap<String, EscrowReservation>,
    seen_transfer_ids: BTreeSet<String>,
    journal: Vec<EscrowJournalEntry>,
}

impl EscrowReservationLedger {
    /// Build an exact partition of the request's quantity budget.
    ///
    /// Allocations must be unique, non-empty replica IDs and sum exactly to the
    /// request quantity. Fixed disjoint allocations allow replicas to reserve
    /// locally without consulting a global coordinator during a partition.
    pub fn new(
        request_id: impl Into<String>,
        requested_quantity: u64,
        allocations: Vec<(String, u64)>,
    ) -> Result<Self, EscrowError> {
        let request_id = request_id.into();
        if request_id.trim().is_empty()
            || requested_quantity == 0
            || allocations.is_empty()
        {
            return Err(EscrowError::InvalidRequest);
        }

        let mut replicas = BTreeMap::new();
        let mut allocated = 0_u64;
        for (replica_id, quantity) in allocations {
            if replica_id.trim().is_empty() {
                return Err(EscrowError::InvalidReplica);
            }
            if replicas.contains_key(&replica_id) {
                return Err(EscrowError::DuplicateReplica);
            }
            allocated = allocated
                .checked_add(quantity)
                .ok_or(EscrowError::ArithmeticOverflow)?;
            replicas.insert(
                replica_id,
                ReplicaBalance {
                    available: quantity,
                    reserved: 0,
                    accepted: 0,
                },
            );
        }
        if allocated != requested_quantity {
            return Err(EscrowError::AllocationMismatch);
        }

        Ok(Self {
            request_id,
            requested_quantity,
            replicas,
            reservations: BTreeMap::new(),
            seen_transfer_ids: BTreeSet::new(),
            journal: Vec::new(),
        })
    }

    pub fn request_id(&self) -> &str {
        &self.request_id
    }

    pub fn requested_quantity(&self) -> u64 {
        self.requested_quantity
    }

    pub fn balance(&self, replica_id: &str) -> Option<&ReplicaBalance> {
        self.replicas.get(replica_id)
    }

    pub fn reservation(&self, claim_id: &str) -> Option<&EscrowReservation> {
        self.reservations.get(claim_id)
    }

    pub fn journal(&self) -> &[EscrowJournalEntry] {
        &self.journal
    }

    pub fn accepted_quantity(&self) -> Result<u64, EscrowError> {
        self.replicas.values().try_fold(0_u64, |total, balance| {
            total
                .checked_add(balance.accepted)
                .ok_or(EscrowError::ArithmeticOverflow)
        })
    }

    pub fn pending_quantity(&self) -> Result<u64, EscrowError> {
        self.replicas.values().try_fold(0_u64, |total, balance| {
            total
                .checked_add(balance.reserved)
                .ok_or(EscrowError::ArithmeticOverflow)
        })
    }

    pub fn available_quantity(&self) -> Result<u64, EscrowError> {
        self.replicas.values().try_fold(0_u64, |total, balance| {
            total
                .checked_add(balance.available)
                .ok_or(EscrowError::ArithmeticOverflow)
        })
    }

    /// Move available escrow rights between replicas.
    ///
    /// This records a transfer ID as single-use within this ledger. It is NOT a
    /// cryptographic proof that two partitioned replicas cannot fork the same
    /// transfer; a distributed implementation must add authenticated, durable
    /// single-spend transfer certificates or disallow transfers while partitioned.
    pub fn transfer_rights(
        &mut self,
        transfer_id: &str,
        from_replica: &str,
        to_replica: &str,
        quantity: u64,
    ) -> Result<(), EscrowError> {
        if transfer_id.trim().is_empty()
            || from_replica.trim().is_empty()
            || to_replica.trim().is_empty()
            || quantity == 0
            || from_replica == to_replica
        {
            return Err(EscrowError::InvalidTransfer);
        }
        if self.seen_transfer_ids.contains(transfer_id) {
            return Err(EscrowError::DuplicateTransfer);
        }
        let from = self
            .replicas
            .get(from_replica)
            .ok_or(EscrowError::UnknownReplica)?;
        let to = self
            .replicas
            .get(to_replica)
            .ok_or(EscrowError::UnknownReplica)?;
        if from.available < quantity {
            return Err(EscrowError::InsufficientRights);
        }
        let from_after = from.available - quantity;
        let to_after = to
            .available
            .checked_add(quantity)
            .ok_or(EscrowError::ArithmeticOverflow)?;

        self.replicas
            .get_mut(from_replica)
            .ok_or(EscrowError::UnknownReplica)?
            .available = from_after;
        self.replicas
            .get_mut(to_replica)
            .ok_or(EscrowError::UnknownReplica)?
            .available = to_after;
        self.seen_transfer_ids.insert(transfer_id.to_owned());
        self.journal.push(EscrowJournalEntry::RightsTransferred {
            transfer_id: transfer_id.to_owned(),
            from_replica: from_replica.to_owned(),
            to_replica: to_replica.to_owned(),
            quantity,
        });
        self.ensure_conservation()?;
        Ok(())
    }

    /// Reserve quantity from one replica's escrow share.
    ///
    /// A pending reservation stays locked even if that replica is disconnected.
    /// A timeout alone must not release it: callers need an authoritative final
    /// disposition to avoid treating an unknown outcome as cancellation.
    pub fn reserve(
        &mut self,
        claim_id: &str,
        replica_id: &str,
        quantity: u64,
    ) -> Result<(), EscrowError> {
        if claim_id.trim().is_empty() || replica_id.trim().is_empty() || quantity == 0 {
            return Err(EscrowError::InvalidRequest);
        }
        if self.reservations.contains_key(claim_id) {
            return Err(EscrowError::DuplicateClaim);
        }
        let balance = self
            .replicas
            .get(replica_id)
            .ok_or(EscrowError::UnknownReplica)?;
        if balance.available < quantity {
            return Err(EscrowError::InsufficientRights);
        }
        let reserved_after = balance
            .reserved
            .checked_add(quantity)
            .ok_or(EscrowError::ArithmeticOverflow)?;
        let available_after = balance.available - quantity;

        let balance = self
            .replicas
            .get_mut(replica_id)
            .ok_or(EscrowError::UnknownReplica)?;
        balance.available = available_after;
        balance.reserved = reserved_after;
        self.reservations.insert(
            claim_id.to_owned(),
            EscrowReservation {
                claim_id: claim_id.to_owned(),
                replica_id: replica_id.to_owned(),
                claimed_quantity: quantity,
                state: ReservationState::Pending,
            },
        );
        self.journal.push(EscrowJournalEntry::ReservationOpened {
            claim_id: claim_id.to_owned(),
            replica_id: replica_id.to_owned(),
            quantity,
        });
        self.ensure_conservation()?;
        Ok(())
    }

    /// Finalize a reservation with an independently established accepted amount.
    ///
    /// The accepted amount is a caller-supplied decision, not authenticated
    /// evidence. A final reservation cannot be replayed or re-counted.
    pub fn finalize(
        &mut self,
        claim_id: &str,
        accepted_quantity: u64,
    ) -> Result<(), EscrowError> {
        let reservation = self
            .reservations
            .get(claim_id)
            .cloned()
            .ok_or(EscrowError::MissingClaim)?;
        if reservation.state != ReservationState::Pending {
            return Err(EscrowError::ClaimAlreadyFinal);
        }
        if accepted_quantity > reservation.claimed_quantity {
            return Err(EscrowError::InvalidAcceptedQuantity);
        }
        let released_quantity = reservation.claimed_quantity - accepted_quantity;
        let balance = self
            .replicas
            .get(&reservation.replica_id)
            .ok_or(EscrowError::UnknownReplica)?;
        if balance.reserved < reservation.claimed_quantity {
            return Err(EscrowError::InvariantViolation);
        }
        let reserved_after = balance.reserved - reservation.claimed_quantity;
        let available_after = balance
            .available
            .checked_add(released_quantity)
            .ok_or(EscrowError::ArithmeticOverflow)?;
        let accepted_after = balance
            .accepted
            .checked_add(accepted_quantity)
            .ok_or(EscrowError::ArithmeticOverflow)?;

        let balance = self
            .replicas
            .get_mut(&reservation.replica_id)
            .ok_or(EscrowError::UnknownReplica)?;
        balance.reserved = reserved_after;
        balance.available = available_after;
        balance.accepted = accepted_after;
        self.reservations
            .get_mut(claim_id)
            .ok_or(EscrowError::MissingClaim)?
            .state = ReservationState::Finalized { accepted_quantity };
        self.journal.push(EscrowJournalEntry::ReservationFinalized {
            claim_id: claim_id.to_owned(),
            replica_id: reservation.replica_id,
            claimed_quantity: reservation.claimed_quantity,
            accepted_quantity,
            released_quantity,
        });
        self.ensure_conservation()?;
        Ok(())
    }

    /// Recompute the core quantity invariant from the current accounting state.
    pub fn conservation_holds(&self) -> bool {
        let mut total = 0_u64;
        for balance in self.replicas.values() {
            let Some(local) = balance
                .available
                .checked_add(balance.reserved)
                .and_then(|value| value.checked_add(balance.accepted))
            else {
                return false;
            };
            let Some(next) = total.checked_add(local) else {
                return false;
            };
            total = next;
        }
        total == self.requested_quantity
    }

    fn ensure_conservation(&self) -> Result<(), EscrowError> {
        if self.conservation_holds() {
            Ok(())
        } else {
            Err(EscrowError::InvariantViolation)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ledger() -> EscrowReservationLedger {
        EscrowReservationLedger::new(
            "request-1",
            10,
            vec![("replica-a".to_owned(), 6), ("replica-b".to_owned(), 4)],
        )
        .unwrap()
    }

    #[test]
    fn initial_allocations_must_partition_the_exact_budget() {
        assert!(matches!(
            EscrowReservationLedger::new("request", 10, vec![("a".to_owned(), 9)]),
            Err(EscrowError::AllocationMismatch)
        ));
        assert!(matches!(
            EscrowReservationLedger::new(
                "request", 10,
                vec![("a".to_owned(), 5), ("a".to_owned(), 5)]
            ),
            Err(EscrowError::DuplicateReplica)
        ));
    }

    #[test]
    fn replica_can_only_reserve_its_disjoint_share() {
        let mut book = ledger();
        book.reserve("a-claim", "replica-a", 6).unwrap();
        assert_eq!(
            book.reserve("b-claim", "replica-a", 1),
            Err(EscrowError::InsufficientRights)
        );
        book.reserve("b-claim", "replica-b", 4).unwrap();
        assert_eq!(book.pending_quantity(), Ok(10));
        assert!(book.conservation_holds());
    }

    #[test]
    fn partial_acceptance_consumes_only_accepted_rights() {
        let mut book = ledger();
        book.reserve("claim", "replica-a", 5).unwrap();
        book.finalize("claim", 3).unwrap();
        assert_eq!(book.accepted_quantity(), Ok(3));
        assert_eq!(book.pending_quantity(), Ok(0));
        assert_eq!(book.balance("replica-a").unwrap().available, 3);
        assert_eq!(book.balance("replica-a").unwrap().accepted, 3);
        assert!(book.conservation_holds());
    }

    #[test]
    fn pending_reservation_is_not_released_by_a_timeout_or_replay() {
        let mut book = ledger();
        book.reserve("claim", "replica-a", 5).unwrap();
        assert_eq!(
            book.reserve("claim", "replica-b", 1),
            Err(EscrowError::DuplicateClaim)
        );
        assert_eq!(book.balance("replica-a").unwrap().reserved, 5);
        assert_eq!(book.finalize("claim", 5), Ok(()));
        assert_eq!(book.finalize("claim", 5), Err(EscrowError::ClaimAlreadyFinal));
    }

    #[test]
    fn transfer_is_bounded_and_single_use_in_one_ledger() {
        let mut book = ledger();
        book.transfer_rights("transfer-1", "replica-a", "replica-b", 2).unwrap();
        assert_eq!(book.balance("replica-a").unwrap().available, 4);
        assert_eq!(book.balance("replica-b").unwrap().available, 6);
        let journal_len = book.journal().len();
        assert_eq!(
            book.transfer_rights("transfer-1", "replica-a", "replica-b", 2),
            Err(EscrowError::DuplicateTransfer)
        );
        assert_eq!(book.journal().len(), journal_len);
        assert!(book.conservation_holds());
    }

    #[test]
    fn invalid_finalization_does_not_mutate_accounting() {
        let mut book = ledger();
        book.reserve("claim", "replica-a", 3).unwrap();
        let before = book.balance("replica-a").unwrap().clone();
        let journal_len = book.journal().len();
        assert_eq!(
            book.finalize("claim", 4),
            Err(EscrowError::InvalidAcceptedQuantity)
        );
        assert_eq!(*book.balance("replica-a").unwrap(), before);
        assert_eq!(book.journal().len(), journal_len);
        assert!(book.conservation_holds());
    }

    #[test]
    fn generated_operation_sequences_preserve_conservation() {
        // Small deterministic state-space campaign. This is a serial model check,
        // not evidence of multi-process or Byzantine safety.
        for seed in 1_u64..=500 {
            let total = seed % 29 + 1;
            let left = total / 2;
            let right = total - left;
            let mut book = EscrowReservationLedger::new(
                format!("request-{seed}"),
                total,
                vec![("a".to_owned(), left), ("b".to_owned(), right)],
            )
            .unwrap();
            let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
            for step in 0_u64..64 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                match state % 3 {
                    0 => {
                        let replica = if (state >> 4) & 1 == 0 { "a" } else { "b" };
                        let qty = (state >> 8) % total + 1;
                        let claim_id = format!("claim-{step}");
                        if book.reserve(&claim_id, replica, qty).is_ok() {
                            let pending = book.reservation(&claim_id).unwrap().claimed_quantity;
                            let accepted = (state >> 16) % (pending + 1);
                            book.finalize(&claim_id, accepted).unwrap();
                        }
                    }
                    1 => {
                        let (from, to) = if (state >> 5) & 1 == 0 {
                            ("a", "b")
                        } else {
                            ("b", "a")
                        };
                        let qty = (state >> 9) % total + 1;
                        let _ = book.transfer_rights(&format!("transfer-{step}"), from, to, qty);
                    }
                    _ => {
                        let _ = book.reserve(
                            &format!("pending-{step}"),
                            if (state >> 3) & 1 == 0 { "a" } else { "b" },
                            (state >> 11) % total + 1,
                        );
                    }
                }
                assert!(
                    book.conservation_holds(),
                    "conservation failed at seed={seed}, step={step}"
                );
                assert!(book.accepted_quantity().unwrap() <= total);
                assert!(book.pending_quantity().unwrap() <= total);
            }
        }
    }
}

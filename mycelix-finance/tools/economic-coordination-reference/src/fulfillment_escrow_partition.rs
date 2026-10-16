//! Multi-replica, fixed-rights simulation for partitioned partial fulfilment.
//!
//! Each simulated replica owns a disjoint quantity share. Local reserve/finalize
//! calls touch only that replica's ledger; no message, transfer or timeout-based
//! release protocol is modeled. The wrapper aggregates state as an independent
//! test oracle. This is a deterministic simulation, not a distributed runtime.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplicaEscrowError {
    InvalidRequest,
    InvalidReplica,
    DuplicateReplica,
    AllocationMismatch,
    UnknownReplica,
    InvalidClaim,
    DuplicateClaim,
    MissingClaim,
    ClaimAlreadyFinal,
    InsufficientRights,
    InvalidAcceptedQuantity,
    ArithmeticOverflow,
    InvariantViolation,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReplicaBalance {
    pub allocation: u64,
    pub available: u64,
    pub reserved: u64,
    pub accepted: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReservationStatus {
    Pending,
    Finalized { accepted_quantity: u64 },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReplicaReservation {
    pub claim_id: String,
    pub replica_id: String,
    pub claimed_quantity: u64,
    pub status: ReservationStatus,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReplicaJournalEntry {
    Reserved {
        claim_id: String,
        replica_id: String,
        quantity: u64,
    },
    Finalized {
        claim_id: String,
        replica_id: String,
        claimed_quantity: u64,
        accepted_quantity: u64,
        released_quantity: u64,
    },
}

#[derive(Debug)]
struct ReplicaLedger {
    balance: ReplicaBalance,
    reservations: BTreeMap<String, ReplicaReservation>,
    seen_claims: BTreeSet<String>,
    journal: Vec<ReplicaJournalEntry>,
}

/// Multi-replica fixed-rights model for one finite request.
///
/// Construction checks that the per-replica allocations form an exact partition
/// of the requested quantity. During a simulated partition, operations are
/// replica-local and no rights transfer is available. This makes safety depend
/// on three explicit assumptions: allocations are uniquely issued, a replica's
/// spent state is not rolled back/forked, and accepted evidence cannot be forged.
/// This module tests the quantity-accounting layer only; it does not establish
/// those distributed assumptions.
#[derive(Debug)]
pub struct PartitionedEscrowModel {
    request_id: String,
    requested_quantity: u64,
    replicas: BTreeMap<String, ReplicaLedger>,
    journal: Vec<ReplicaJournalEntry>,
}

impl PartitionedEscrowModel {
    pub fn new(
        request_id: impl Into<String>,
        requested_quantity: u64,
        allocations: Vec<(String, u64)>,
    ) -> Result<Self, ReplicaEscrowError> {
        let request_id = request_id.into();
        if request_id.trim().is_empty() || requested_quantity == 0 || allocations.is_empty() {
            return Err(ReplicaEscrowError::InvalidRequest);
        }
        let mut total = 0_u64;
        let mut replicas = BTreeMap::new();
        for (replica_id, allocation) in allocations {
            if replica_id.trim().is_empty() {
                return Err(ReplicaEscrowError::InvalidReplica);
            }
            if replicas.contains_key(&replica_id) {
                return Err(ReplicaEscrowError::DuplicateReplica);
            }
            total = total
                .checked_add(allocation)
                .ok_or(ReplicaEscrowError::ArithmeticOverflow)?;
            replicas.insert(
                replica_id,
                ReplicaLedger {
                    balance: ReplicaBalance {
                        allocation,
                        available: allocation,
                        reserved: 0,
                        accepted: 0,
                    },
                    reservations: BTreeMap::new(),
                    seen_claims: BTreeSet::new(),
                    journal: Vec::new(),
                },
            );
        }
        if total != requested_quantity {
            return Err(ReplicaEscrowError::AllocationMismatch);
        }
        Ok(Self {
            request_id,
            requested_quantity,
            replicas,
            journal: Vec::new(),
        })
    }

    pub fn request_id(&self) -> &str {
        &self.request_id
    }

    pub fn requested_quantity(&self) -> u64 {
        self.requested_quantity
    }

    pub fn replica_ids(&self) -> impl Iterator<Item = &str> {
        self.replicas.keys().map(String::as_str)
    }

    pub fn balance(&self, replica_id: &str) -> Option<&ReplicaBalance> {
        self.replicas.get(replica_id).map(|replica| &replica.balance)
    }

    pub fn reservation(&self, replica_id: &str, claim_id: &str) -> Option<&ReplicaReservation> {
        self.replicas.get(replica_id)?.reservations.get(claim_id)
    }

    pub fn journal(&self) -> &[ReplicaJournalEntry] {
        &self.journal
    }

    pub fn accepted_quantity(&self) -> Result<u64, ReplicaEscrowError> {
        self.replicas.values().try_fold(0_u64, |sum, replica| {
            sum.checked_add(replica.balance.accepted)
                .ok_or(ReplicaEscrowError::ArithmeticOverflow)
        })
    }

    pub fn pending_quantity(&self) -> Result<u64, ReplicaEscrowError> {
        self.replicas.values().try_fold(0_u64, |sum, replica| {
            sum.checked_add(replica.balance.reserved)
                .ok_or(ReplicaEscrowError::ArithmeticOverflow)
        })
    }

    pub fn available_quantity(&self) -> Result<u64, ReplicaEscrowError> {
        self.replicas.values().try_fold(0_u64, |sum, replica| {
            sum.checked_add(replica.balance.available)
                .ok_or(ReplicaEscrowError::ArithmeticOverflow)
        })
    }

    /// Reserve from only the chosen replica's preallocated share.
    ///
    /// The call neither reads nor mutates another replica's balance.
    pub fn reserve(
        &mut self,
        replica_id: &str,
        claim_id: &str,
        quantity: u64,
    ) -> Result<(), ReplicaEscrowError> {
        if claim_id.trim().is_empty() || quantity == 0 {
            return Err(ReplicaEscrowError::InvalidClaim);
        }
        let replica = self
            .replicas
            .get_mut(replica_id)
            .ok_or(ReplicaEscrowError::UnknownReplica)?;
        if replica.seen_claims.contains(claim_id) {
            return Err(ReplicaEscrowError::DuplicateClaim);
        }
        if replica.balance.available < quantity {
            return Err(ReplicaEscrowError::InsufficientRights);
        }

        let available_after = replica.balance.available - quantity;
        let reserved_after = replica
            .balance
            .reserved
            .checked_add(quantity)
            .ok_or(ReplicaEscrowError::ArithmeticOverflow)?;
        replica.balance.available = available_after;
        replica.balance.reserved = reserved_after;
        replica.seen_claims.insert(claim_id.to_owned());
        let entry = ReplicaJournalEntry::Reserved {
            claim_id: claim_id.to_owned(),
            replica_id: replica_id.to_owned(),
            quantity,
        };
        replica.journal.push(entry.clone());
        replica.reservations.insert(
            claim_id.to_owned(),
            ReplicaReservation {
                claim_id: claim_id.to_owned(),
                replica_id: replica_id.to_owned(),
                claimed_quantity: quantity,
                status: ReservationStatus::Pending,
            },
        );
        self.journal.push(entry);
        self.ensure_invariants()
    }

    /// Finalize a pending reservation; zero acceptance releases all its rights.
    /// The caller's accepted quantity is an input, not authenticated evidence.
    pub fn finalize(
        &mut self,
        replica_id: &str,
        claim_id: &str,
        accepted_quantity: u64,
    ) -> Result<(), ReplicaEscrowError> {
        let replica = self
            .replicas
            .get(replica_id)
            .ok_or(ReplicaEscrowError::UnknownReplica)?;
        let reservation = replica
            .reservations
            .get(claim_id)
            .cloned()
            .ok_or(ReplicaEscrowError::MissingClaim)?;
        if reservation.status != ReservationStatus::Pending {
            return Err(ReplicaEscrowError::ClaimAlreadyFinal);
        }
        if accepted_quantity > reservation.claimed_quantity {
            return Err(ReplicaEscrowError::InvalidAcceptedQuantity);
        }
        if replica.balance.reserved < reservation.claimed_quantity {
            return Err(ReplicaEscrowError::InvariantViolation);
        }
        let released = reservation.claimed_quantity - accepted_quantity;
        let reserved_after = replica.balance.reserved - reservation.claimed_quantity;
        let available_after = replica
            .balance
            .available
            .checked_add(released)
            .ok_or(ReplicaEscrowError::ArithmeticOverflow)?;
        let accepted_after = replica
            .balance
            .accepted
            .checked_add(accepted_quantity)
            .ok_or(ReplicaEscrowError::ArithmeticOverflow)?;

        let replica = self
            .replicas
            .get_mut(replica_id)
            .ok_or(ReplicaEscrowError::UnknownReplica)?;
        replica.balance.reserved = reserved_after;
        replica.balance.available = available_after;
        replica.balance.accepted = accepted_after;
        replica
            .reservations
            .get_mut(claim_id)
            .ok_or(ReplicaEscrowError::MissingClaim)?
            .status = ReservationStatus::Finalized { accepted_quantity };
        let entry = ReplicaJournalEntry::Finalized {
            claim_id: claim_id.to_owned(),
            replica_id: replica_id.to_owned(),
            claimed_quantity: reservation.claimed_quantity,
            accepted_quantity,
            released_quantity: released,
        };
        replica.journal.push(entry.clone());
        self.journal.push(entry);
        self.ensure_invariants()
    }

    /// Independent aggregate oracle across every simulated replica.
    pub fn invariants_hold(&self) -> bool {
        let mut allocated = 0_u64;
        for replica in self.replicas.values() {
            let Some(local_total) = replica
                .balance
                .available
                .checked_add(replica.balance.reserved)
                .and_then(|n| n.checked_add(replica.balance.accepted))
            else {
                return false;
            };
            if local_total != replica.balance.allocation {
                return false;
            }
            let Some(next) = allocated.checked_add(local_total) else {
                return false;
            };
            allocated = next;
        }
        allocated == self.requested_quantity
            && self.accepted_quantity().is_ok_and(|q| q <= self.requested_quantity)
    }

    fn ensure_invariants(&self) -> Result<(), ReplicaEscrowError> {
        if self.invariants_hold() {
            Ok(())
        } else {
            Err(ReplicaEscrowError::InvariantViolation)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> PartitionedEscrowModel {
        PartitionedEscrowModel::new(
            "request-1",
            10,
            vec![
                ("replica-a".to_owned(), 6),
                ("replica-b".to_owned(), 4),
            ],
        )
        .unwrap()
    }

    #[test]
    fn allocation_must_be_an_exact_partition() {
        assert_eq!(
            PartitionedEscrowModel::new("request", 10, vec![("a".into(), 9)]).err(),
            Some(ReplicaEscrowError::AllocationMismatch)
        );
        assert_eq!(
            PartitionedEscrowModel::new(
                "request",
                10,
                vec![("a".into(), 5), ("a".into(), 5)]
            ).err(),
            Some(ReplicaEscrowError::DuplicateReplica)
        );
    }

    #[test]
    fn replica_local_reservations_cannot_exceed_disjoint_rights() {
        let mut world = model();
        world.reserve("replica-a", "a-claim", 6).unwrap();
        world.reserve("replica-b", "b-claim", 4).unwrap();
        assert_eq!(world.accepted_quantity(), Ok(0));
        assert_eq!(world.pending_quantity(), Ok(10));
        assert_eq!(
            world.reserve("replica-a", "a-over", 1),
            Err(ReplicaEscrowError::InsufficientRights)
        );
        assert_eq!(
            world.reserve("replica-b", "b-over", 1),
            Err(ReplicaEscrowError::InsufficientRights)
        );
        assert!(world.invariants_hold());
    }

    #[test]
    fn concurrent_in_the_trace_partial_acceptance_preserves_global_bound() {
        let mut world = model();
        world.reserve("replica-a", "a-claim", 5).unwrap();
        world.reserve("replica-b", "b-claim", 4).unwrap();
        world.finalize("replica-a", "a-claim", 3).unwrap();
        world.finalize("replica-b", "b-claim", 4).unwrap();
        assert_eq!(world.accepted_quantity(), Ok(7));
        assert_eq!(world.available_quantity(), Ok(3));
        assert_eq!(world.pending_quantity(), Ok(0));
        assert!(world.invariants_hold());
    }

    #[test]
    fn pending_claims_are_not_released_by_timeout_semantics() {
        let mut world = model();
        world.reserve("replica-a", "a-pending", 6).unwrap();
        assert_eq!(world.balance("replica-a").unwrap().reserved, 6);
        assert_eq!(world.balance("replica-a").unwrap().available, 0);
        // There is deliberately no timeout or implicit release API.
        assert_eq!(
            world.finalize("replica-a", "missing-claim", 0),
            Err(ReplicaEscrowError::MissingClaim)
        );
        assert!(world.invariants_hold());
    }

    #[test]
    fn finalized_claim_cannot_be_replayed() {
        let mut world = model();
        world.reserve("replica-a", "a-claim", 5).unwrap();
        world.finalize("replica-a", "a-claim", 3).unwrap();
        assert_eq!(
            world.finalize("replica-a", "a-claim", 5),
            Err(ReplicaEscrowError::ClaimAlreadyFinal)
        );
        assert_eq!(world.accepted_quantity(), Ok(3));
        assert!(world.invariants_hold());
    }

    #[test]
    fn invalid_finalization_does_not_mutate_state() {
        let mut world = model();
        world.reserve("replica-a", "a-claim", 4).unwrap();
        let before = world.balance("replica-a").unwrap().clone();
        let journal_len = world.journal().len();
        assert_eq!(
            world.finalize("replica-a", "a-claim", 5),
            Err(ReplicaEscrowError::InvalidAcceptedQuantity)
        );
        assert_eq!(*world.balance("replica-a").unwrap(), before);
        assert_eq!(world.journal().len(), journal_len);
    }

    #[test]
    fn duplicated_allocation_plan_is_a_known_global_overacceptance_counterexample() {
        // Two independent ledgers each believe they own the same initial grants.
        // Both are locally valid, but their combined accepted quantity exceeds
        // the single request budget. This is an expected negative control.
        let mut first = model();
        let mut forked = model();
        first.reserve("replica-a", "delivery-a", 6).unwrap();
        first.finalize("replica-a", "delivery-a", 6).unwrap();
        forked.reserve("replica-a", "delivery-b", 6).unwrap();
        forked.finalize("replica-a", "delivery-b", 6).unwrap();

        assert!(first.invariants_hold());
        assert!(forked.invariants_hold());
        assert_eq!(first.accepted_quantity(), Ok(6));
        assert_eq!(forked.accepted_quantity(), Ok(6));
        let combined_accepted =
            first.accepted_quantity().unwrap() + forked.accepted_quantity().unwrap();
        assert_eq!(combined_accepted, 12);
        assert!(combined_accepted > first.requested_quantity());
    }

    #[test]
    fn same_external_claim_replayed_across_replicas_is_a_known_idempotency_counterexample() {
        // The local claim-ID set is per replica. Quantity escrow still bounds
        // the request, but it does not deduplicate a single external delivery
        // identity across replicas. A successor must bind each delivery to a
        // unique, single-use reservation/evidence identity.
        let mut world = model();
        world.reserve("replica-a", "external-delivery-42", 3).unwrap();
        world.finalize("replica-a", "external-delivery-42", 3).unwrap();
        world.reserve("replica-b", "external-delivery-42", 3).unwrap();
        world.finalize("replica-b", "external-delivery-42", 3).unwrap();

        assert_eq!(world.accepted_quantity(), Ok(6));
        assert!(world.invariants_hold());
        assert_eq!(
            world.journal().iter().filter(|entry| match entry {
                ReplicaJournalEntry::Finalized { claim_id, accepted_quantity, .. } =>
                    claim_id == "external-delivery-42" && *accepted_quantity == 3,
                _ => false,
            }).count(),
            2
        );
    }

    #[test]
    fn deterministic_interleaving_campaign_preserves_global_conservation() {
        for seed in 1_u64..=1_000 {
            let total = seed % 97 + 1;
            let replica_count = (seed as usize % 7) + 2;
            let mut allocations = vec![0_u64; replica_count];
            for unit in 0..total {
                allocations[unit as usize % replica_count] += 1;
            }
            let mut world = PartitionedEscrowModel::new(
                format!("request-{seed}"),
                total,
                allocations
                    .into_iter()
                    .enumerate()
                    .map(|(index, quantity)| (format!("replica-{index}"), quantity))
                    .collect(),
            )
            .unwrap();
            let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
            for step in 0_u64..128 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let replica_index = (state as usize) % replica_count;
                let replica_id = format!("replica-{replica_index}");
                let claim_id = format!("claim-{seed}-{step}");
                let quantity = (state.rotate_left(17) % total) + 1;
                if world.reserve(&replica_id, &claim_id, quantity).is_ok() {
                    let accepted = state.rotate_left(31) % (quantity + 1);
                    world.finalize(&replica_id, &claim_id, accepted).unwrap();
                }
                assert!(
                    world.invariants_hold(),
                    "seed={seed}, step={step}, replica={replica_id}"
                );
            }
        }
    }
}

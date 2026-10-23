//! Partition-safe gross-exposure budget reservations.
//!
//! This reference model allocates disjoint capacity slices before a partition. A
//! shard may use only its own reservation; settlement releases capacity only inside
//! that same reservation. Reservations are not reclaimed/reassigned automatically:
//! safe rebalancing requires a separately qualified fencing/reconciliation protocol.

use std::collections::BTreeMap;

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ReservationKey {
    pub agreement_id: String,
    pub instrument: String,
    pub shard_id: String,
    /// Monotonic allocation epoch. Old epochs retain their own allocation until a
    /// future, explicitly qualified retirement/fencing protocol retires them.
    pub epoch: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Reservation {
    community_a: String,
    community_b: String,
    capacity: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
enum BudgetEvent {
    Obligation { key: ReservationKey, amount: u64 },
    Settlement { key: ReservationKey, amount: u64 },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReservationError {
    InvalidKey,
    InvalidAmount,
    MissingCommunityCap,
    MissingAgreementCap,
    CapacityExceeded,
    ReservationConflict,
    UnknownReservation,
    ReplayMismatch,
    NothingOutstanding,
    SettlementTooLarge,
    Overflow,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReservationApply {
    Reserved,
    Applied,
    Duplicate,
}

#[derive(Default)]
pub struct ReservationBook {
    community_caps: BTreeMap<(String, String), u64>,
    agreement_caps: BTreeMap<(String, String), u64>,
    reservations: BTreeMap<ReservationKey, Reservation>,
    outstanding: BTreeMap<ReservationKey, u64>,
    /// Event IDs are globally unique within this reference ledger. A distributed
    /// deployment must route IDs to a stable owner or use a qualified global ID scheme.
    events: BTreeMap<String, BudgetEvent>,
}

impl ReservationBook {
    pub fn set_community_cap(
        &mut self,
        community: &str,
        instrument: &str,
        cap: u64,
    ) -> Result<(), ReservationError> {
        if community.trim().is_empty() || instrument.trim().is_empty() {
            return Err(ReservationError::InvalidKey);
        }
        if self.allocated_to_community(community, instrument)? > cap {
            return Err(ReservationError::CapacityExceeded);
        }
        self.community_caps.insert((community.into(), instrument.into()), cap);
        Ok(())
    }

    pub fn set_agreement_cap(
        &mut self,
        agreement_id: &str,
        instrument: &str,
        cap: u64,
    ) -> Result<(), ReservationError> {
        if agreement_id.trim().is_empty() || instrument.trim().is_empty() {
            return Err(ReservationError::InvalidKey);
        }
        if self.allocated_to_agreement(agreement_id, instrument)? > cap {
            return Err(ReservationError::CapacityExceeded);
        }
        self.agreement_caps.insert((agreement_id.into(), instrument.into()), cap);
        Ok(())
    }

    /// Reserve a disjoint slice against both community caps and the agreement cap.
    /// A reservation is pinned to one shard/owner and epoch; independent shards never
    /// receive the same units of risk capacity.
    pub fn reserve(
        &mut self,
        key: ReservationKey,
        community_a: &str,
        community_b: &str,
        capacity: u64,
    ) -> Result<ReservationApply, ReservationError> {
        if key.agreement_id.trim().is_empty()
            || key.instrument.trim().is_empty()
            || key.shard_id.trim().is_empty()
            || key.epoch == 0
            || community_a.trim().is_empty()
            || community_b.trim().is_empty()
            || community_a == community_b
        {
            return Err(ReservationError::InvalidKey);
        }
        if capacity == 0 {
            return Err(ReservationError::InvalidAmount);
        }

        let candidate = Reservation {
            community_a: community_a.into(),
            community_b: community_b.into(),
            capacity,
        };
        if let Some(existing) = self.reservations.get(&key) {
            return if existing == &candidate {
                Ok(ReservationApply::Duplicate)
            } else {
                Err(ReservationError::ReservationConflict)
            };
        }

        for community in [community_a, community_b] {
            if !self.community_caps.contains_key(&(community.into(), key.instrument.clone())) {
                return Err(ReservationError::MissingCommunityCap);
            }
        }
        if !self.agreement_caps.contains_key(&(key.agreement_id.clone(), key.instrument.clone())) {
            return Err(ReservationError::MissingAgreementCap);
        }

        for community in [community_a, community_b] {
            let cap = self.community_caps[&(community.into(), key.instrument.clone())];
            let allocated = self.allocated_to_community(community, &key.instrument)?;
            if allocated.checked_add(capacity).ok_or(ReservationError::Overflow)? > cap {
                return Err(ReservationError::CapacityExceeded);
            }
        }

        let agreement_cap = self.agreement_caps[&(key.agreement_id.clone(), key.instrument.clone())];
        let agreement_allocated = self.allocated_to_agreement(&key.agreement_id, &key.instrument)?;
        if agreement_allocated.checked_add(capacity).ok_or(ReservationError::Overflow)? > agreement_cap {
            return Err(ReservationError::CapacityExceeded);
        }

        self.outstanding.insert(key.clone(), 0);
        self.reservations.insert(key, candidate);
        Ok(ReservationApply::Reserved)
    }

    /// Record gross new exposure inside one pre-allocated reservation.
    /// Opposite-direction obligations still consume gross capacity; this model does not net.
    pub fn record_obligation(
        &mut self,
        event_id: &str,
        key: &ReservationKey,
        amount: u64,
    ) -> Result<ReservationApply, ReservationError> {
        if event_id.trim().is_empty() {
            return Err(ReservationError::InvalidKey);
        }
        if amount == 0 {
            return Err(ReservationError::InvalidAmount);
        }
        let event = BudgetEvent::Obligation { key: key.clone(), amount };
        if let Some(old) = self.events.get(event_id) {
            return if old == &event {
                Ok(ReservationApply::Duplicate)
            } else {
                Err(ReservationError::ReplayMismatch)
            };
        }
        let reservation = self.reservations.get(key).ok_or(ReservationError::UnknownReservation)?;
        let current = self.outstanding.get(key).copied().ok_or(ReservationError::UnknownReservation)?;
        let next = current.checked_add(amount).ok_or(ReservationError::Overflow)?;
        if next > reservation.capacity {
            return Err(ReservationError::CapacityExceeded);
        }
        self.outstanding.insert(key.clone(), next);
        self.events.insert(event_id.into(), event);
        Ok(ReservationApply::Applied)
    }

    /// Settle an amount against its reservation. Capacity becomes reusable by the
    /// same reservation only; this does not transfer risk rights to another shard.
    pub fn settle(
        &mut self,
        event_id: &str,
        key: &ReservationKey,
        amount: u64,
    ) -> Result<ReservationApply, ReservationError> {
        if event_id.trim().is_empty() {
            return Err(ReservationError::InvalidKey);
        }
        if amount == 0 {
            return Err(ReservationError::InvalidAmount);
        }
        let event = BudgetEvent::Settlement { key: key.clone(), amount };
        if let Some(old) = self.events.get(event_id) {
            return if old == &event {
                Ok(ReservationApply::Duplicate)
            } else {
                Err(ReservationError::ReplayMismatch)
            };
        }
        if !self.reservations.contains_key(key) {
            return Err(ReservationError::UnknownReservation);
        }
        let current = self.outstanding.get(key).copied().ok_or(ReservationError::UnknownReservation)?;
        if current == 0 {
            return Err(ReservationError::NothingOutstanding);
        }
        if amount > current {
            return Err(ReservationError::SettlementTooLarge);
        }
        let next = current.checked_sub(amount).ok_or(ReservationError::Overflow)?;
        self.outstanding.insert(key.clone(), next);
        self.events.insert(event_id.into(), event);
        Ok(ReservationApply::Applied)
    }

    pub fn reservation_capacity(&self, key: &ReservationKey) -> Result<u64, ReservationError> {
        self.reservations.get(key)
            .map(|reservation| reservation.capacity)
            .ok_or(ReservationError::UnknownReservation)
    }

    pub fn outstanding(&self, key: &ReservationKey) -> Result<u64, ReservationError> {
        self.outstanding.get(key).copied().ok_or(ReservationError::UnknownReservation)
    }

    pub fn allocated_to_community(
        &self,
        community: &str,
        instrument: &str,
    ) -> Result<u64, ReservationError> {
        let mut total = 0_u64;
        for (key, reservation) in &self.reservations {
            if key.instrument == instrument
                && (reservation.community_a == community || reservation.community_b == community)
            {
                total = total.checked_add(reservation.capacity).ok_or(ReservationError::Overflow)?;
            }
        }
        Ok(total)
    }

    pub fn allocated_to_agreement(
        &self,
        agreement_id: &str,
        instrument: &str,
    ) -> Result<u64, ReservationError> {
        let mut total = 0_u64;
        for (key, reservation) in &self.reservations {
            if key.agreement_id == agreement_id && key.instrument == instrument {
                total = total.checked_add(reservation.capacity).ok_or(ReservationError::Overflow)?;
            }
        }
        Ok(total)
    }

    /// Total gross outstanding for one community and one exact instrument.
    pub fn outstanding_for_community(
        &self,
        community: &str,
        instrument: &str,
    ) -> Result<u64, ReservationError> {
        let mut total = 0_u64;
        for (key, reservation) in &self.reservations {
            if key.instrument == instrument
                && (reservation.community_a == community || reservation.community_b == community)
            {
                total = total.checked_add(self.outstanding(key)?).ok_or(ReservationError::Overflow)?;
            }
        }
        Ok(total)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(agreement_id: &str, instrument: &str, shard_id: &str, epoch: u64) -> ReservationKey {
        ReservationKey {
            agreement_id: agreement_id.into(),
            instrument: instrument.into(),
            shard_id: shard_id.into(),
            epoch,
        }
    }

    fn configured_book(community_cap: u64, agreement_cap: u64) -> ReservationBook {
        let mut book = ReservationBook::default();
        for community in ["a", "b"] {
            book.set_community_cap(community, "unit:local", community_cap).unwrap();
        }
        book.set_agreement_cap("edge:ab", "unit:local", agreement_cap).unwrap();
        book
    }

    #[test]
    fn reservations_require_explicit_caps_and_are_idempotent() {
        let mut book = ReservationBook::default();
        assert_eq!(
            book.reserve(key("edge:ab", "unit:local", "shard-1", 1), "a", "b", 40),
            Err(ReservationError::MissingCommunityCap)
        );

        let mut book = configured_book(100, 100);
        let k1 = key("edge:ab", "unit:local", "shard-1", 1);
        assert_eq!(book.reserve(k1.clone(), "a", "b", 60), Ok(ReservationApply::Reserved));
        assert_eq!(book.reserve(k1.clone(), "a", "b", 60), Ok(ReservationApply::Duplicate));
        assert_eq!(book.reserve(k1, "a", "b", 61), Err(ReservationError::ReservationConflict));
        assert_eq!(
            book.reserve(key("edge:ab", "unit:local", "shard-2", 1), "a", "b", 41),
            Err(ReservationError::CapacityExceeded)
        );
    }

    #[test]
    fn partitioned_shards_can_consume_only_disjoint_preallocated_slices() {
        let mut book = configured_book(100, 100);
        let k1 = key("edge:ab", "unit:local", "shard-1", 1);
        let k2 = key("edge:ab", "unit:local", "shard-2", 1);
        book.reserve(k1.clone(), "a", "b", 40).unwrap();
        book.reserve(k2.clone(), "a", "b", 60).unwrap();

        assert_eq!(book.record_obligation("tx:a-40", &k1, 40), Ok(ReservationApply::Applied));
        assert_eq!(book.record_obligation("tx:a-over", &k1, 1), Err(ReservationError::CapacityExceeded));
        assert_eq!(book.outstanding(&k1), Ok(40));
        assert_eq!(book.record_obligation("tx:b-60", &k2, 60), Ok(ReservationApply::Applied));
        assert_eq!(book.record_obligation("tx:b-over", &k2, 1), Err(ReservationError::CapacityExceeded));
        assert_eq!(book.outstanding_for_community("a", "unit:local"), Ok(100));
        assert_eq!(book.outstanding_for_community("b", "unit:local"), Ok(100));
        assert_eq!(book.allocated_to_community("a", "unit:local"), Ok(100));
    }

    #[test]
    fn settlement_releases_capacity_only_within_the_same_reservation() {
        let mut book = configured_book(100, 100);
        let k1 = key("edge:ab", "unit:local", "shard-1", 1);
        let k2 = key("edge:ab", "unit:local", "shard-2", 1);
        book.reserve(k1.clone(), "a", "b", 40).unwrap();
        book.reserve(k2.clone(), "a", "b", 60).unwrap();
        book.record_obligation("tx:1", &k1, 40).unwrap();
        assert_eq!(book.settle("settle:1", &k1, 40), Ok(ReservationApply::Applied));
        assert_eq!(book.outstanding(&k1), Ok(0));
        assert_eq!(book.outstanding(&k2), Ok(0));
        // Settlement on shard 1 cannot transfer its released 40-unit slice to shard 2.
        assert_eq!(
            book.record_obligation("tx:wrong-shard", &k2, 61),
            Err(ReservationError::CapacityExceeded)
        );
        assert_eq!(book.record_obligation("tx:2", &k1, 40), Ok(ReservationApply::Applied));
        assert_eq!(book.outstanding_for_community("a", "unit:local"), Ok(40));
    }

    #[test]
    fn different_instruments_have_independent_budget_domains() {
        let mut book = ReservationBook::default();
        for community in ["a", "b"] {
            book.set_community_cap(community, "unit:hours", 50).unwrap();
            book.set_community_cap(community, "unit:water", 80).unwrap();
        }
        book.set_agreement_cap("edge:hours", "unit:hours", 50).unwrap();
        book.set_agreement_cap("edge:water", "unit:water", 80).unwrap();
        assert_eq!(
            book.reserve(key("edge:hours", "unit:hours", "shard-1", 1), "a", "b", 50),
            Ok(ReservationApply::Reserved)
        );
        assert_eq!(
            book.reserve(key("edge:water", "unit:water", "shard-1", 1), "a", "b", 80),
            Ok(ReservationApply::Reserved)
        );
        assert_eq!(book.allocated_to_community("a", "unit:hours"), Ok(50));
        assert_eq!(book.allocated_to_community("a", "unit:water"), Ok(80));
    }

    #[test]
    fn zero_community_or_agreement_cap_prevents_new_reservations() {
        let mut book = ReservationBook::default();
        book.set_community_cap("a", "unit:local", 0).unwrap();
        book.set_community_cap("b", "unit:local", 100).unwrap();
        book.set_agreement_cap("edge:ab", "unit:local", 100).unwrap();
        assert_eq!(
            book.reserve(key("edge:ab", "unit:local", "shard-1", 1), "a", "b", 1),
            Err(ReservationError::CapacityExceeded)
        );

        book.set_community_cap("a", "unit:local", 100).unwrap();
        book.set_agreement_cap("edge:ab", "unit:local", 0).unwrap();
        assert_eq!(
            book.reserve(key("edge:ab", "unit:local", "shard-1", 1), "a", "b", 1),
            Err(ReservationError::CapacityExceeded)
        );
    }

    #[test]
    fn event_id_cannot_be_replayed_against_a_different_reservation() {
        let mut book = configured_book(100, 100);
        let k1 = key("edge:ab", "unit:local", "shard-1", 1);
        let k2 = key("edge:ab", "unit:local", "shard-2", 1);
        book.reserve(k1.clone(), "a", "b", 40).unwrap();
        book.reserve(k2.clone(), "a", "b", 60).unwrap();
        assert_eq!(book.record_obligation("tx:global", &k1, 10), Ok(ReservationApply::Applied));
        assert_eq!(
            book.record_obligation("tx:global", &k2, 10),
            Err(ReservationError::ReplayMismatch)
        );
        assert_eq!(book.outstanding(&k1), Ok(10));
        assert_eq!(book.outstanding(&k2), Ok(0));
    }

    #[test]
    fn old_epoch_capacity_is_not_reused_without_rebalancing() {
        let mut book = configured_book(100, 100);
        let old = key("edge:ab", "unit:local", "shard-1", 1);
        let new = key("edge:ab", "unit:local", "shard-1", 2);
        book.reserve(old.clone(), "a", "b", 60).unwrap();
        assert_eq!(book.reserve(new.clone(), "a", "b", 41), Err(ReservationError::CapacityExceeded));
        book.reserve(new.clone(), "a", "b", 40).unwrap();

        // Separate epochs have disjoint allocations. Unused/released capacity from the
        // old epoch is not reclaimed automatically while an old owner may still be active.
        book.record_obligation("tx:old", &old, 60).unwrap();
        book.record_obligation("tx:new", &new, 40).unwrap();
        assert_eq!(book.outstanding_for_community("a", "unit:local"), Ok(100));
    }
}

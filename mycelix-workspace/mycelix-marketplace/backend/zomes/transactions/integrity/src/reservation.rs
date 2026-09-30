// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure inventory reservation algebra.
//!
//! This module deliberately contains no Holochain host calls. It is the
//! deterministic state machine that higher layers can use to reduce an
//! append-only reservation/consumption evidence stream.
//!
//! It does not, by itself, provide distributed admission. Two agents can
//! independently observe the same capacity and both propose reservations.
//! A network-level authority/frontier protocol must decide which reservation
//! evidence is admissible. Once an ordered evidence set is supplied, this
//! reducer has one deterministic result.

use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reservation {
    pub intent_id: String,
    pub listing_revision: String,
    pub quantity: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationState {
    Active(Reservation),
    Released(Reservation),
    Consumed(Reservation),
}

impl ReservationState {
    fn reservation(&self) -> &Reservation {
        match self {
            Self::Active(r) | Self::Released(r) | Self::Consumed(r) => r,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationEvent {
    Reserve(Reservation),
    Release { intent_id: String },
    Consume { intent_id: String },
    SetCapacity { capacity: u32 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationError {
    ZeroQuantity,
    EmptyIntentId,
    EmptyListingRevision,
    CapacityBelowActiveReservations { capacity: u32, reserved: u32 },
    UnknownIntent { intent_id: String },
    IntentConflict { intent_id: String },
    AlreadyConsumed { intent_id: String },
    AlreadyReleased { intent_id: String },
    Overflow,
    InsufficientCapacity { requested: u32, available: u32 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ApplyOutcome {
    Applied,
    Idempotent,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationLedger {
    capacity: u32,
    reservations: BTreeMap<String, ReservationState>,
}

impl ReservationLedger {
    pub fn new(capacity: u32) -> Self {
        Self {
            capacity,
            reservations: BTreeMap::new(),
        }
    }

    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    pub fn active_reserved(&self) -> u32 {
        self.reservations
            .values()
            .filter_map(|state| match state {
                ReservationState::Active(reservation) => Some(reservation.quantity),
                ReservationState::Released(_) | ReservationState::Consumed(_) => None,
            })
            .sum()
    }

    pub fn available(&self) -> u32 {
        self.capacity - self.active_reserved()
    }

    pub fn state(&self, intent_id: &str) -> Option<&ReservationState> {
        self.reservations.get(intent_id)
    }

    pub fn apply(&mut self, event: ReservationEvent) -> Result<ApplyOutcome, ReservationError> {
        match event {
            ReservationEvent::Reserve(reservation) => self.reserve(reservation),
            ReservationEvent::Release { intent_id } => self.transition_terminal(&intent_id, true),
            ReservationEvent::Consume { intent_id } => self.transition_terminal(&intent_id, false),
            ReservationEvent::SetCapacity { capacity } => self.set_capacity(capacity),
        }
    }

    fn reserve(&mut self, reservation: Reservation) -> Result<ApplyOutcome, ReservationError> {
        if reservation.intent_id.trim().is_empty() {
            return Err(ReservationError::EmptyIntentId);
        }
        if reservation.listing_revision.trim().is_empty() {
            return Err(ReservationError::EmptyListingRevision);
        }
        if reservation.quantity == 0 {
            return Err(ReservationError::ZeroQuantity);
        }

        if let Some(existing) = self.reservations.get(&reservation.intent_id) {
            if existing.reservation() == &reservation {
                return Ok(ApplyOutcome::Idempotent);
            }
            return Err(ReservationError::IntentConflict {
                intent_id: reservation.intent_id,
            });
        }

        let available = self.available();
        if reservation.quantity > available {
            return Err(ReservationError::InsufficientCapacity {
                requested: reservation.quantity,
                available,
            });
        }

        self.reservations.insert(
            reservation.intent_id.clone(),
            ReservationState::Active(reservation),
        );
        Ok(ApplyOutcome::Applied)
    }

    fn transition_terminal(
        &mut self,
        intent_id: &str,
        release: bool,
    ) -> Result<ApplyOutcome, ReservationError> {
        let state = self
            .reservations
            .get_mut(intent_id)
            .ok_or_else(|| ReservationError::UnknownIntent {
                intent_id: intent_id.to_owned(),
            })?;

        match state {
            ReservationState::Active(reservation) => {
                let reservation = reservation.clone();
                if release {
                    *state = ReservationState::Released(reservation);
                } else {
                    // Consumption transfers the reserved quantity out of the
                    // remaining inventory capacity. Release returns capacity;
                    // consume permanently removes it.
                    self.capacity = self
                        .capacity
                        .checked_sub(reservation.quantity)
                        .ok_or(ReservationError::Overflow)?;
                    *state = ReservationState::Consumed(reservation);
                }
                Ok(ApplyOutcome::Applied)
            }
            ReservationState::Released(_) if release => Ok(ApplyOutcome::Idempotent),
            ReservationState::Consumed(_) if !release => Ok(ApplyOutcome::Idempotent),
            ReservationState::Released(_) => Err(ReservationError::AlreadyReleased {
                intent_id: intent_id.to_owned(),
            }),
            ReservationState::Consumed(_) => Err(ReservationError::AlreadyConsumed {
                intent_id: intent_id.to_owned(),
            }),
        }
    }

    fn set_capacity(&mut self, capacity: u32) -> Result<ApplyOutcome, ReservationError> {
        let reserved = self.active_reserved();
        if capacity < reserved {
            return Err(ReservationError::CapacityBelowActiveReservations {
                capacity,
                reserved,
            });
        }

        if capacity == self.capacity {
            Ok(ApplyOutcome::Idempotent)
        } else {
            self.capacity = capacity;
            Ok(ApplyOutcome::Applied)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reservation(intent: &str, revision: &str, quantity: u32) -> Reservation {
        Reservation {
            intent_id: intent.into(),
            listing_revision: revision.into(),
            quantity,
        }
    }

    #[test]
    fn reserve_consumes_capacity_and_release_restores_it() {
        let mut ledger = ReservationLedger::new(10);
        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(reservation("i1", "r1", 3))),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(ledger.active_reserved(), 3);
        assert_eq!(ledger.available(), 7);

        assert_eq!(
            ledger.apply(ReservationEvent::Release { intent_id: "i1".into() }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(ledger.active_reserved(), 0);
        assert_eq!(ledger.available(), 10);
    }

    #[test]
    fn reservation_is_bound_to_exact_intent_and_revision() {
        let mut ledger = ReservationLedger::new(10);
        let r = reservation("i1", "r1", 2);
        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(r.clone())),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(r)),
            Ok(ApplyOutcome::Idempotent)
        );

        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(reservation("i1", "r2", 2))),
            Err(ReservationError::IntentConflict { intent_id: "i1".into() })
        );
    }

    #[test]
    fn capacity_cannot_drop_below_outstanding_reservations() {
        let mut ledger = ReservationLedger::new(10);
        ledger.apply(ReservationEvent::Reserve(reservation("i1", "r1", 6))).unwrap();

        assert_eq!(
            ledger.apply(ReservationEvent::SetCapacity { capacity: 5 }),
            Err(ReservationError::CapacityBelowActiveReservations {
                capacity: 5,
                reserved: 6
            })
        );
        assert_eq!(ledger.capacity(), 10);
    }

    #[test]
    fn capacity_increase_is_allowed() {
        let mut ledger = ReservationLedger::new(3);
        ledger.apply(ReservationEvent::Reserve(reservation("i1", "r1", 3))).unwrap();

        assert_eq!(
            ledger.apply(ReservationEvent::SetCapacity { capacity: 8 }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(ledger.available(), 5);
    }

    #[test]
    fn duplicate_terminal_event_is_idempotent_but_cross_terminal_is_conflict() {
        let mut ledger = ReservationLedger::new(4);
        ledger.apply(ReservationEvent::Reserve(reservation("i1", "r1", 2))).unwrap();

        assert_eq!(
            ledger.apply(ReservationEvent::Consume { intent_id: "i1".into() }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(
            ledger.apply(ReservationEvent::Consume { intent_id: "i1".into() }),
            Ok(ApplyOutcome::Idempotent)
        );
        assert_eq!(
            ledger.apply(ReservationEvent::Release { intent_id: "i1".into() }),
            Err(ReservationError::AlreadyConsumed { intent_id: "i1".into() })
        );
    }

    #[test]
    fn cannot_reserve_more_than_available() {
        let mut ledger = ReservationLedger::new(5);
        ledger.apply(ReservationEvent::Reserve(reservation("i1", "r1", 4))).unwrap();

        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(reservation("i2", "r1", 2))),
            Err(ReservationError::InsufficientCapacity {
                requested: 2,
                available: 1
            })
        );
    }

    #[test]
    fn invalid_reservations_are_rejected() {
        let mut ledger = ReservationLedger::new(5);
        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(reservation("", "r1", 1))),
            Err(ReservationError::EmptyIntentId)
        );
        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(reservation("i1", "", 1))),
            Err(ReservationError::EmptyListingRevision)
        );
        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(reservation("i1", "r1", 0))),
            Err(ReservationError::ZeroQuantity)
        );
    }

    #[test]
    fn deterministic_reduction_is_order_sensitive_by_design() {
        let first = [
            ReservationEvent::Reserve(reservation("i1", "r1", 3)),
            ReservationEvent::Reserve(reservation("i2", "r1", 2)),
        ];
        let second = [
            ReservationEvent::Reserve(reservation("i2", "r1", 2)),
            ReservationEvent::Reserve(reservation("i1", "r1", 3)),
        ];

        let mut a = ReservationLedger::new(4);
        let mut b = ReservationLedger::new(4);
        let first_results: Vec<_> = first.into_iter().map(|event| a.apply(event)).collect();
        let second_results: Vec<_> = second.into_iter().map(|event| b.apply(event)).collect();

        assert_eq!(first_results[0], Ok(ApplyOutcome::Applied));
        assert_eq!(
            first_results[1],
            Err(ReservationError::InsufficientCapacity { requested: 2, available: 1 })
        );
        assert_eq!(second_results[0], Ok(ApplyOutcome::Applied));
        assert_eq!(
            second_results[1],
            Err(ReservationError::InsufficientCapacity { requested: 3, available: 2 })
        );
        assert_eq!(a.active_reserved(), b.active_reserved());
        assert_eq!(a.active_reserved(), 3);
    }

    #[test]
    fn consumed_inventory_is_removed_from_capacity() {
        let mut ledger = ReservationLedger::new(2);
        ledger.apply(ReservationEvent::Reserve(reservation("i1", "r1", 2))).unwrap();
        ledger.apply(ReservationEvent::Consume { intent_id: "i1".into() }).unwrap();

        assert_eq!(ledger.capacity(), 0);
        assert_eq!(ledger.available(), 0);
        assert_eq!(
            ledger.apply(ReservationEvent::Reserve(reservation("i2", "r1", 1))),
            Err(ReservationError::InsufficientCapacity {
                requested: 1,
                available: 0
            })
        );
    }
}

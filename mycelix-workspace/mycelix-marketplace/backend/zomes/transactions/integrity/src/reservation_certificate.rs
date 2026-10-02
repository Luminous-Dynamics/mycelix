// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Seller-authoritative inventory frontier algebra.
//!
//! This module is deliberately host-independent. It defines the semantic
//! objects that the Holochain coordinator/integrity layer can later bind to
//! source-chain actions and addressable DHT dependencies.
//!
//! The seller's source chain is the intended serialization point for
//! ReservationCertificate admission. The algebra itself does not provide a
//! network lock or distributed consensus; it makes the authority frontier
//! explicit and deterministic once an ordered seller-authored certificate
//! stream is supplied.

use hdi::prelude::*;
use listings_integrity::Listing;
use std::collections::BTreeMap;

use crate::reservation::{ApplyOutcome, Reservation, ReservationError, ReservationLedger};

#[hdk_entry_helper]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PurchaseIntent {
    pub intent_id: String,
    pub buyer: AgentPubKey,
    pub seller: AgentPubKey,
    pub listing_hash: ActionHash,
    pub listing_revision: ActionHash,
    pub quantity: u32,
    pub unit_price_cents: u64,
    pub client_nonce: String,
}

impl PurchaseIntent {
    pub fn validate(&self) -> Result<(), IntentError> {
        if self.intent_id.trim().is_empty() {
            return Err(IntentError::EmptyIntentId);
        }
        if self.client_nonce.trim().is_empty() {
            return Err(IntentError::EmptyClientNonce);
        }
        if self.quantity == 0 {
            return Err(IntentError::ZeroQuantity);
        }
        if self.buyer == self.seller {
            return Err(IntentError::BuyerSellerMustDiffer);
        }
        Ok(())
    }

    /// Canonical, domain-separated identity material for semantic intent IDs.
    ///
    /// The coordinator should hash these exact bytes with the protocol's
    /// approved digest to obtain intent_id. Length-prefixing prevents
    /// delimiter-collision ambiguity, and all binary identifiers are encoded
    /// from their raw 36-byte Holochain representation.
    pub fn canonical_identity_material(&self) -> Vec<u8> {
        let mut out = Vec::new();
        push_field(&mut out, b"mycelix.marketplace.purchase-intent/v1");
        push_field(&mut out, &self.buyer.get_raw_36());
        push_field(&mut out, &self.seller.get_raw_36());
        push_field(&mut out, &self.listing_hash.get_raw_36());
        push_field(&mut out, &self.listing_revision.get_raw_36());
        push_field(&mut out, &self.quantity.to_le_bytes());
        push_field(&mut out, &self.unit_price_cents.to_le_bytes());
        push_field(&mut out, self.client_nonce.as_bytes());
        out
    }
}

fn push_field(out: &mut Vec<u8>, field: &[u8]) {
    out.extend_from_slice(&(field.len() as u32).to_le_bytes());
    out.extend_from_slice(field);
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IntentError {
    EmptyIntentId,
    EmptyClientNonce,
    ZeroQuantity,
    BuyerSellerMustDiffer,
}

#[hdk_entry_helper]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationCertificate {
    pub certificate_id: String,
    pub seller: AgentPubKey,
    pub listing_hash: ActionHash,
    pub listing_revision: ActionHash,
    pub intent_hash: ActionHash,
    pub intent: PurchaseIntent,
    pub quantity: u32,
    pub sequence: u64,
    pub previous_certificate_id: Option<String>,
    pub previous_frontier_action: Option<ActionHash>,
    /// Economic state immediately before this reservation event.
    pub pre_state: ReservationFrontierState,
    /// Economic state immediately after this reservation event.
    pub post_state: ReservationFrontierState,
}

impl ReservationCertificate {
    pub fn validate(&self) -> Result<(), CertificateError> {
        if self.certificate_id.trim().is_empty() {
            return Err(CertificateError::EmptyCertificateId);
        }
        if self.quantity == 0 {
            return Err(CertificateError::ZeroQuantity);
        }
        self.intent
            .validate()
            .map_err(CertificateError::InvalidIntent)?;
        if self.seller != self.intent.seller {
            return Err(CertificateError::SellerMismatch);
        }
        if self.listing_hash != self.intent.listing_hash {
            return Err(CertificateError::ListingMismatch);
        }
        if self.listing_revision != self.intent.listing_revision {
            return Err(CertificateError::ListingRevisionMismatch);
        }
        if self.quantity != self.intent.quantity {
            return Err(CertificateError::QuantityMismatch);
        }
        if self.sequence == 0
            && (self.previous_certificate_id.is_some() || self.previous_frontier_action.is_some())
        {
            return Err(CertificateError::GenesisHasPrevious);
        }
        if self.sequence > 0
            && (self.previous_certificate_id.is_none() || self.previous_frontier_action.is_none())
        {
            return Err(CertificateError::MissingPrevious);
        }
        self.pre_state
            .validate()
            .map_err(|_| CertificateError::InvalidPreState)?;
        self.post_state
            .validate()
            .map_err(|_| CertificateError::InvalidPostState)?;
        self.pre_state
            .validate_transition(
                FrontierStateTransition::Reserve {
                    quantity: self.quantity,
                },
                &self.post_state,
            )
            .map_err(|_| CertificateError::InvalidStateTransition)?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CertificateError {
    EmptyCertificateId,
    ZeroQuantity,
    InvalidIntent(IntentError),
    SellerMismatch,
    ListingMismatch,
    ListingRevisionMismatch,
    QuantityMismatch,
    GenesisHasPrevious,
    MissingPrevious,
    InvalidPreState,
    InvalidPostState,
    InvalidStateTransition,
    WrongSeller,
    WrongListing,
    WrongRevision,
    SequenceMismatch {
        expected: u64,
        actual: u64,
    },
    PreviousMismatch {
        expected: Option<String>,
        actual: Option<String>,
    },
    IntentConflict,
    Capacity(ReservationError),
    UnknownCertificate(String),
    AlreadyReleased(String),
    AlreadyConsumed(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrontierEvent {
    Reserve(ReservationCertificate),
    Release {
        certificate_id: String,
        seller: AgentPubKey,
        sequence: u64,
        previous_certificate_id: Option<String>,
    },
    Consume {
        certificate_id: String,
        seller: AgentPubKey,
        sequence: u64,
        previous_certificate_id: Option<String>,
    },
    SetCapacity {
        listing_hash: ActionHash,
        listing_revision: ActionHash,
        capacity: u32,
        seller: AgentPubKey,
        sequence: u64,
        previous_certificate_id: Option<String>,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InventoryFrontier {
    seller: AgentPubKey,
    listing_hash: ActionHash,
    listing_revision: ActionHash,
    ledger: ReservationLedger,
    next_sequence: u64,
    head_certificate_id: Option<String>,
    certificates: BTreeMap<String, ReservationCertificate>,
    terminal_events: BTreeMap<(String, bool), (u64, Option<String>)>,
}

/// Deterministic snapshot of the economic state represented by a frontier.
///
/// This is intentionally structural rather than cryptographic. On the current
/// Holochain 0.6 baseline, the authoritative content-addressed predecessor
/// remains the action hash; this snapshot gives independent reducers an exact
/// post-state to compare without introducing an unsupported arbitrary-byte hash
/// dependency.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReservationFrontierState {
    pub capacity: u32,
    pub active_reserved: u32,
    pub available: u32,
}

impl ReservationFrontierState {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.active_reserved > self.capacity {
            return Err("Frontier state has more reserved inventory than capacity");
        }
        if self.available != self.capacity - self.active_reserved {
            return Err("Frontier state available capacity is inconsistent");
        }
        Ok(())
    }

    pub fn from_capacity(capacity: u32) -> Self {
        Self {
            capacity,
            active_reserved: 0,
            available: capacity,
        }
    }

    pub fn after_reserve(&self, quantity: u32) -> Result<Self, &'static str> {
        if quantity == 0 {
            return Err("Reservation quantity must be non-zero");
        }
        if quantity > self.available {
            return Err("Reservation exceeds available capacity");
        }
        Ok(Self {
            capacity: self.capacity,
            active_reserved: self.active_reserved + quantity,
            available: self.available - quantity,
        })
    }

    pub fn after_release(&self, quantity: u32) -> Result<Self, &'static str> {
        if quantity == 0 {
            return Err("Reservation quantity must be non-zero");
        }
        if quantity > self.active_reserved {
            return Err("Release exceeds active reserved quantity");
        }
        Ok(Self {
            capacity: self.capacity,
            active_reserved: self.active_reserved - quantity,
            available: self.available + quantity,
        })
    }

    pub fn after_consume(&self, quantity: u32) -> Result<Self, &'static str> {
        if quantity == 0 {
            return Err("Reservation quantity must be non-zero");
        }
        if quantity > self.active_reserved || quantity > self.capacity {
            return Err("Consumption exceeds active inventory state");
        }
        Ok(Self {
            capacity: self.capacity - quantity,
            active_reserved: self.active_reserved - quantity,
            available: self.available,
        })
    }

    pub fn after_set_capacity(&self, capacity: u32) -> Result<Self, &'static str> {
        if capacity < self.active_reserved {
            return Err("Capacity cannot fall below active reservations");
        }
        Ok(Self {
            capacity,
            active_reserved: self.active_reserved,
            available: capacity - self.active_reserved,
        })
    }

    pub fn validate_transition(
        &self,
        event: FrontierStateTransition,
        post: &Self,
    ) -> Result<(), &'static str> {
        self.validate()?;
        post.validate()?;
        let expected = match event {
            FrontierStateTransition::Reserve { quantity } => self.after_reserve(quantity)?,
            FrontierStateTransition::Release { quantity } => self.after_release(quantity)?,
            FrontierStateTransition::Consume { quantity } => self.after_consume(quantity)?,
            FrontierStateTransition::SetCapacity { capacity } => {
                self.after_set_capacity(capacity)?
            }
        };
        if &expected != post {
            return Err("Frontier post-state does not match the deterministic transition");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrontierStateTransition {
    Reserve { quantity: u32 },
    Release { quantity: u32 },
    Consume { quantity: u32 },
    SetCapacity { capacity: u32 },
}

impl InventoryFrontier {
    pub fn new(
        seller: AgentPubKey,
        listing_hash: ActionHash,
        listing_revision: ActionHash,
        capacity: u32,
    ) -> Self {
        Self {
            seller,
            listing_hash,
            listing_revision,
            ledger: ReservationLedger::new(capacity),
            next_sequence: 0,
            head_certificate_id: None,
            certificates: BTreeMap::new(),
            terminal_events: BTreeMap::new(),
        }
    }

    pub fn seller(&self) -> &AgentPubKey {
        &self.seller
    }
    pub fn listing_hash(&self) -> &ActionHash {
        &self.listing_hash
    }
    pub fn listing_revision(&self) -> &ActionHash {
        &self.listing_revision
    }
    pub fn capacity(&self) -> u32 {
        self.ledger.capacity()
    }
    pub fn available(&self) -> u32 {
        self.ledger.available()
    }
    pub fn active_reserved(&self) -> u32 {
        self.ledger.active_reserved()
    }
    pub fn post_state(&self) -> ReservationFrontierState {
        ReservationFrontierState {
            capacity: self.capacity(),
            active_reserved: self.active_reserved(),
            available: self.available(),
        }
    }
    pub fn next_sequence(&self) -> u64 {
        self.next_sequence
    }
    pub fn head_certificate_id(&self) -> Option<&str> {
        self.head_certificate_id.as_deref()
    }
    pub fn certificate(&self, certificate_id: &str) -> Option<&ReservationCertificate> {
        self.certificates.get(certificate_id)
    }

    pub fn apply(&mut self, event: FrontierEvent) -> Result<ApplyOutcome, CertificateError> {
        match event {
            FrontierEvent::Reserve(certificate) => self.reserve(certificate),
            FrontierEvent::Release {
                certificate_id,
                seller,
                sequence,
                previous_certificate_id,
            } => self.terminal(
                &certificate_id,
                seller,
                sequence,
                previous_certificate_id,
                true,
            ),
            FrontierEvent::Consume {
                certificate_id,
                seller,
                sequence,
                previous_certificate_id,
            } => self.terminal(
                &certificate_id,
                seller,
                sequence,
                previous_certificate_id,
                false,
            ),
            FrontierEvent::SetCapacity {
                listing_hash,
                listing_revision,
                capacity,
                seller,
                sequence,
                previous_certificate_id,
            } => self.set_capacity(
                listing_hash,
                listing_revision,
                capacity,
                seller,
                sequence,
                previous_certificate_id,
            ),
        }
    }

    fn check_frontier(
        &self,
        seller: &AgentPubKey,
        sequence: u64,
        previous_certificate_id: &Option<String>,
    ) -> Result<(), CertificateError> {
        if seller != &self.seller {
            return Err(CertificateError::WrongSeller);
        }
        if sequence != self.next_sequence {
            return Err(CertificateError::SequenceMismatch {
                expected: self.next_sequence,
                actual: sequence,
            });
        }
        if previous_certificate_id != &self.head_certificate_id {
            return Err(CertificateError::PreviousMismatch {
                expected: self.head_certificate_id.clone(),
                actual: previous_certificate_id.clone(),
            });
        }
        Ok(())
    }

    fn reserve(
        &mut self,
        certificate: ReservationCertificate,
    ) -> Result<ApplyOutcome, CertificateError> {
        certificate.validate()?;
        if certificate.seller != self.seller {
            return Err(CertificateError::WrongSeller);
        }
        if certificate.listing_hash != self.listing_hash {
            return Err(CertificateError::WrongListing);
        }
        if certificate.listing_revision != self.listing_revision {
            return Err(CertificateError::WrongRevision);
        }
        if let Some(existing) = self.certificates.get(&certificate.certificate_id) {
            if existing == &certificate {
                return Ok(ApplyOutcome::Idempotent);
            }
            return Err(CertificateError::IntentConflict);
        }
        self.check_frontier(
            &certificate.seller,
            certificate.sequence,
            &certificate.previous_certificate_id,
        )?;

        // The certificate carries its own deterministic transition evidence.
        // Cross-event continuity is validated against the predecessor evidence
        // at the Holochain boundary; the reducer remains reusable for replay and
        // simulation without requiring a particular initial frontier snapshot.

        let reservation = Reservation {
            intent_id: certificate.intent.intent_id.clone(),
            listing_revision: format!("{:?}", certificate.listing_revision),
            quantity: certificate.quantity,
        };
        let outcome = self
            .ledger
            .apply(crate::reservation::ReservationEvent::Reserve(reservation))
            .map_err(CertificateError::Capacity)?;

        self.certificates
            .insert(certificate.certificate_id.clone(), certificate.clone());
        self.advance(certificate.certificate_id);
        Ok(outcome)
    }

    fn terminal(
        &mut self,
        certificate_id: &str,
        seller: AgentPubKey,
        sequence: u64,
        previous_certificate_id: Option<String>,
        release: bool,
    ) -> Result<ApplyOutcome, CertificateError> {
        let certificate = self
            .certificates
            .get(certificate_id)
            .ok_or_else(|| CertificateError::UnknownCertificate(certificate_id.to_owned()))?;

        let terminal_key = (certificate_id.to_owned(), release);
        if let Some((recorded_sequence, recorded_previous)) =
            self.terminal_events.get(&terminal_key)
        {
            if *recorded_sequence == sequence && *recorded_previous == previous_certificate_id {
                return Ok(ApplyOutcome::Idempotent);
            }
            return Err(CertificateError::IntentConflict);
        }

        let expected_head = format!(
            "{}:{}",
            certificate_id,
            if release { "release" } else { "consume" }
        );
        if self.head_certificate_id.as_deref() == Some(expected_head.as_str())
            && sequence.checked_add(1) == Some(self.next_sequence)
            && previous_certificate_id.as_deref() == Some(certificate_id)
        {
            return Ok(ApplyOutcome::Idempotent);
        }

        self.check_frontier(&seller, sequence, &previous_certificate_id)?;
        if certificate.seller != self.seller {
            return Err(CertificateError::WrongSeller);
        }

        let event = if release {
            crate::reservation::ReservationEvent::Release {
                intent_id: certificate.intent.intent_id.clone(),
            }
        } else {
            crate::reservation::ReservationEvent::Consume {
                intent_id: certificate.intent.intent_id.clone(),
            }
        };

        let outcome = self.ledger.apply(event).map_err(|error| match error {
            ReservationError::AlreadyReleased { .. } => {
                CertificateError::AlreadyReleased(certificate_id.to_owned())
            }
            ReservationError::AlreadyConsumed { .. } => {
                CertificateError::AlreadyConsumed(certificate_id.to_owned())
            }
            other => CertificateError::Capacity(other),
        })?;

        self.terminal_events
            .insert(terminal_key, (sequence, previous_certificate_id.clone()));
        self.advance(format!(
            "{}:{}",
            certificate_id,
            if release { "release" } else { "consume" }
        ));
        Ok(outcome)
    }

    fn set_capacity(
        &mut self,
        listing_hash: ActionHash,
        listing_revision: ActionHash,
        capacity: u32,
        seller: AgentPubKey,
        sequence: u64,
        previous_certificate_id: Option<String>,
    ) -> Result<ApplyOutcome, CertificateError> {
        self.check_frontier(&seller, sequence, &previous_certificate_id)?;
        if listing_hash != self.listing_hash {
            return Err(CertificateError::WrongListing);
        }

        let outcome = self
            .ledger
            .apply(crate::reservation::ReservationEvent::SetCapacity { capacity })
            .map_err(CertificateError::Capacity)?;
        // Capacity evidence is also the frontier bridge to a new seller-owned
        // listing revision. The Holochain validator independently proves that
        // this revision is the current root-derived revision before admitting it.
        self.listing_revision = listing_revision;
        self.advance(format!("capacity:{}", sequence));
        Ok(outcome)
    }

    fn advance(&mut self, certificate_id: String) {
        self.head_certificate_id = Some(certificate_id);
        self.next_sequence += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }
    fn hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn intent() -> PurchaseIntent {
        PurchaseIntent {
            intent_id: "intent-1".into(),
            buyer: agent(1),
            seller: agent(2),
            listing_hash: hash(3),
            listing_revision: hash(4),
            quantity: 1,
            unit_price_cents: 500,
            client_nonce: "nonce-1".into(),
        }
    }

    fn certificate(
        id: &str,
        sequence: u64,
        previous: Option<&str>,
        quantity: u32,
    ) -> ReservationCertificate {
        certificate_with_capacity(id, sequence, previous, quantity, 2)
    }

    fn certificate_with_capacity(
        id: &str,
        sequence: u64,
        previous: Option<&str>,
        quantity: u32,
        capacity: u32,
    ) -> ReservationCertificate {
        let intent = PurchaseIntent {
            quantity,
            ..intent()
        };
        ReservationCertificate {
            certificate_id: id.into(),
            seller: agent(2),
            listing_hash: hash(3),
            listing_revision: hash(4),
            intent_hash: hash(5),
            quantity,
            sequence,
            previous_certificate_id: previous.map(str::to_owned),
            previous_frontier_action: previous.map(|_| hash(6)),
            pre_state: ReservationFrontierState::from_capacity(capacity),
            post_state: ReservationFrontierState::from_capacity(capacity)
                .after_reserve(quantity)
                .unwrap(),
            intent,
        }
    }

    fn frontier(capacity: u32) -> InventoryFrontier {
        InventoryFrontier::new(agent(2), hash(3), hash(4), capacity)
    }

    #[test]
    fn seller_frontier_serializes_two_competing_reservations() {
        let mut f = frontier(1);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(certificate_with_capacity(
                "c1", 0, None, 1, 1
            ))),
            Ok(ApplyOutcome::Applied)
        );
        assert!(matches!(
            f.apply(FrontierEvent::Reserve(certificate_with_capacity(
                "c2",
                1,
                Some("c1"),
                1,
                1
            ))),
            Err(CertificateError::Capacity(
                ReservationError::InsufficientCapacity { .. }
            ))
        ));
        assert_eq!(f.active_reserved(), 1);
        assert_eq!(f.available(), 0);
    }

    #[test]
    fn replay_of_exact_certificate_is_idempotent() {
        let mut f = frontier(2);
        let c1 = certificate("c1", 0, None, 1);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(c1.clone())),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(
            f.apply(FrontierEvent::Reserve(c1)),
            Ok(ApplyOutcome::Idempotent)
        );
        assert_eq!(f.active_reserved(), 1);
    }

    #[test]
    fn canonical_intent_identity_material_is_stable_and_domain_separated() {
        let a = intent();
        let mut b = a.clone();
        b.client_nonce = "nonce-2".into();

        assert_eq!(
            a.canonical_identity_material(),
            a.canonical_identity_material()
        );
        assert_ne!(
            a.canonical_identity_material(),
            b.canonical_identity_material()
        );
        assert!(
            a.canonical_identity_material()
                .starts_with(&36u32.to_le_bytes())
        );
    }

    #[test]
    fn certificate_binds_seller_listing_revision_and_quantity() {
        let mut f = frontier(2);
        let mut bad = certificate("c1", 0, None, 1);
        bad.seller = agent(9);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(bad)),
            Err(CertificateError::WrongSeller)
        );

        let mut bad = certificate("c1", 0, None, 1);
        bad.listing_revision = hash(9);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(bad)),
            Err(CertificateError::ListingRevisionMismatch)
        );
    }

    #[test]
    fn listing_capacity_cannot_drop_below_outstanding_reservations() {
        let mut f = frontier(5);
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 4)))
            .unwrap();
        assert!(matches!(
            f.apply(FrontierEvent::SetCapacity {
                listing_hash: hash(3),
                listing_revision: hash(4),
                capacity: 3,
                seller: agent(2),
                sequence: 1,
                previous_certificate_id: Some("c1".into()),
            }),
            Err(CertificateError::Capacity(
                ReservationError::CapacityBelowActiveReservations { .. }
            ))
        ));
        assert_eq!(f.capacity(), 5);
    }

    #[test]
    fn release_returns_capacity_then_next_reservation_can_admit() {
        let mut f = frontier(1);
        f.apply(FrontierEvent::Reserve(certificate_with_capacity(
            "c1", 0, None, 1, 1,
        )))
        .unwrap();
        f.apply(FrontierEvent::Release {
            certificate_id: "c1".into(),
            seller: agent(2),
            sequence: 1,
            previous_certificate_id: Some("c1".into()),
        })
        .unwrap();
        assert_eq!(f.available(), 1);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(certificate_with_capacity(
                "c2",
                2,
                Some("c1:release"),
                1,
                1
            ))),
            Ok(ApplyOutcome::Applied)
        );
    }

    #[test]
    fn frontier_state_transition_algebra_matches_all_inventory_events() {
        let genesis = ReservationFrontierState::from_capacity(10);

        let reserved = ReservationFrontierState {
            capacity: 10,
            active_reserved: 3,
            available: 7,
        };
        genesis
            .validate_transition(FrontierStateTransition::Reserve { quantity: 3 }, &reserved)
            .unwrap();

        let released = ReservationFrontierState {
            capacity: 10,
            active_reserved: 0,
            available: 10,
        };
        reserved
            .validate_transition(FrontierStateTransition::Release { quantity: 3 }, &released)
            .unwrap();

        let consumed = ReservationFrontierState {
            capacity: 7,
            active_reserved: 0,
            available: 7,
        };
        reserved
            .validate_transition(FrontierStateTransition::Consume { quantity: 3 }, &consumed)
            .unwrap();

        let increased = ReservationFrontierState {
            capacity: 15,
            active_reserved: 3,
            available: 12,
        };
        reserved
            .validate_transition(
                FrontierStateTransition::SetCapacity { capacity: 15 },
                &increased,
            )
            .unwrap();
    }

    #[test]
    fn frontier_state_transition_rejects_forged_post_state() {
        let before = ReservationFrontierState::from_capacity(5);
        let forged = ReservationFrontierState {
            capacity: 5,
            active_reserved: 3,
            available: 3,
        };

        assert!(
            before
                .validate_transition(FrontierStateTransition::Reserve { quantity: 1 }, &forged,)
                .is_err()
        );
    }

    #[test]
    fn frontier_post_state_is_self_consistent_and_reconstructible() {
        let mut f = frontier(5);
        assert_eq!(
            f.post_state(),
            ReservationFrontierState {
                capacity: 5,
                active_reserved: 0,
                available: 5
            }
        );
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 3)))
            .unwrap();
        assert_eq!(
            f.post_state(),
            ReservationFrontierState {
                capacity: 5,
                active_reserved: 3,
                available: 2
            }
        );
        f.post_state().validate().unwrap();
        f.apply(FrontierEvent::Release {
            certificate_id: "c1".into(),
            seller: agent(2),
            sequence: 1,
            previous_certificate_id: Some("c1".into()),
        })
        .unwrap();
        assert_eq!(
            f.post_state(),
            ReservationFrontierState {
                capacity: 5,
                active_reserved: 0,
                available: 5
            }
        );
    }

    #[derive(Debug, Clone)]
    struct ReconstructionEvent {
        sequence: u64,
        previous_id: Option<&'static str>,
        id: &'static str,
        transition: FrontierStateTransition,
        pre_state: ReservationFrontierState,
        post_state: ReservationFrontierState,
    }

    fn reconstruct_frontier(
        genesis: ReservationFrontierState,
        events: &[ReconstructionEvent],
    ) -> Result<ReservationFrontierState, &'static str> {
        let mut state = genesis;
        let mut expected_sequence = 0;
        let mut previous_id = None;

        for event in events {
            if event.sequence != expected_sequence {
                return Err("frontier sequence gap");
            }
            if event.previous_id != previous_id {
                return Err("frontier predecessor mismatch");
            }
            if event.pre_state != state {
                return Err("frontier pre-state mismatch");
            }
            state.validate_transition(event.transition, &event.post_state)?;
            state = event.post_state.clone();
            expected_sequence = expected_sequence
                .checked_add(1)
                .ok_or("frontier sequence overflow")?;
            previous_id = Some(event.id);
        }

        Ok(state)
    }

    #[test]
    fn independent_reconstruction_matches_mixed_frontier_history() {
        let genesis = ReservationFrontierState::from_capacity(10);
        let reserved = genesis.after_reserve(4).unwrap();
        let increased = reserved.after_set_capacity(12).unwrap();
        let released = increased.after_release(4).unwrap();
        let consumed_capacity = released.after_reserve(3).unwrap();
        let consumed = consumed_capacity.after_consume(3).unwrap();

        let events = vec![
            ReconstructionEvent {
                sequence: 0,
                previous_id: None,
                id: "reserve-1",
                transition: FrontierStateTransition::Reserve { quantity: 4 },
                pre_state: genesis.clone(),
                post_state: reserved.clone(),
            },
            ReconstructionEvent {
                sequence: 1,
                previous_id: Some("reserve-1"),
                id: "capacity-1",
                transition: FrontierStateTransition::SetCapacity { capacity: 12 },
                pre_state: reserved.clone(),
                post_state: increased.clone(),
            },
            ReconstructionEvent {
                sequence: 2,
                previous_id: Some("capacity-1"),
                id: "release-1",
                transition: FrontierStateTransition::Release { quantity: 4 },
                pre_state: increased.clone(),
                post_state: released.clone(),
            },
            ReconstructionEvent {
                sequence: 3,
                previous_id: Some("release-1"),
                id: "reserve-2",
                transition: FrontierStateTransition::Reserve { quantity: 3 },
                pre_state: released.clone(),
                post_state: consumed_capacity.clone(),
            },
            ReconstructionEvent {
                sequence: 4,
                previous_id: Some("reserve-2"),
                id: "consume-2",
                transition: FrontierStateTransition::Consume { quantity: 3 },
                pre_state: consumed_capacity,
                post_state: consumed.clone(),
            },
        ];

        assert_eq!(reconstruct_frontier(genesis, &events).unwrap(), consumed);
    }

    #[test]
    fn independent_reconstruction_rejects_tampered_state_and_sequence_gaps() {
        let genesis = ReservationFrontierState::from_capacity(5);
        let reserved = genesis.after_reserve(2).unwrap();

        let mut tampered = vec![ReconstructionEvent {
            sequence: 0,
            previous_id: None,
            id: "reserve-1",
            transition: FrontierStateTransition::Reserve { quantity: 2 },
            pre_state: genesis.clone(),
            post_state: ReservationFrontierState {
                capacity: 5,
                active_reserved: 1,
                available: 4,
            },
        }];
        assert_eq!(
            reconstruct_frontier(genesis.clone(), &tampered),
            Err("frontier post-state does not match the deterministic transition")
        );

        tampered[0].post_state = reserved.clone();
        tampered.push(ReconstructionEvent {
            sequence: 2,
            previous_id: Some("reserve-1"),
            id: "reserve-2",
            transition: FrontierStateTransition::Reserve { quantity: 1 },
            pre_state: reserved.clone(),
            post_state: reserved.after_reserve(1).unwrap(),
        });
        assert_eq!(
            reconstruct_frontier(genesis, &tampered),
            Err("frontier sequence gap")
        );
    }

    #[test]
    fn listing_revision_bridge_allows_new_revision_after_capacity_evidence() {
        let mut f = frontier(2);
        let first = certificate_with_capacity("c1", 0, None, 1, 2);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(first)),
            Ok(ApplyOutcome::Applied)
        );

        let new_revision = hash(7);
        assert_eq!(
            f.apply(FrontierEvent::SetCapacity {
                listing_hash: hash(3),
                listing_revision: new_revision.clone(),
                capacity: 3,
                seller: agent(2),
                sequence: 1,
                previous_certificate_id: Some("c1".into()),
            }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(f.listing_revision(), &new_revision);

        let mut second = certificate_with_capacity("c2", 2, Some("capacity:1"), 1, 3);
        second.listing_revision = new_revision;
        second.intent.listing_revision = second.listing_revision.clone();
        assert_eq!(
            f.apply(FrontierEvent::Reserve(second)),
            Ok(ApplyOutcome::Applied)
        );
    }

    #[test]
    fn historical_reservation_can_terminate_after_listing_revision_bridge() {
        let mut f = frontier(2);
        let first = certificate_with_capacity("c1", 0, None, 1, 2);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(first)),
            Ok(ApplyOutcome::Applied)
        );

        let new_revision = hash(7);
        assert_eq!(
            f.apply(FrontierEvent::SetCapacity {
                listing_hash: hash(3),
                listing_revision: new_revision.clone(),
                capacity: 2,
                seller: agent(2),
                sequence: 1,
                previous_certificate_id: Some("c1".into()),
            }),
            Ok(ApplyOutcome::Applied)
        );

        assert_eq!(
            f.apply(FrontierEvent::Release {
                certificate_id: "c1".into(),
                seller: agent(2),
                sequence: 2,
                previous_certificate_id: Some("capacity:1".into()),
            }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(f.active_reserved(), 0);
        assert_eq!(f.available(), 2);
        assert_eq!(f.listing_revision(), &new_revision);
    }

    #[test]
    fn capacity_revision_cannot_reduce_below_existing_reservations() {
        let mut f = frontier(3);
        let first = certificate_with_capacity("c1", 0, None, 2, 3);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(first)),
            Ok(ApplyOutcome::Applied)
        );

        let error = f
            .apply(FrontierEvent::SetCapacity {
                listing_hash: hash(3),
                listing_revision: hash(8),
                capacity: 1,
                seller: agent(2),
                sequence: 1,
                previous_certificate_id: Some("c1".into()),
            })
            .unwrap_err();

        assert!(matches!(
            error,
            CertificateError::Capacity(ReservationError::CapacityBelowActive { .. })
        ));
        assert_eq!(f.active_reserved(), 2);
        assert_eq!(f.capacity(), 3);
        assert_eq!(f.listing_revision(), &hash(4));
    }

    #[test]
    fn historical_release_is_valid_after_listing_revision_bridge() {
        let mut f = frontier(2);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(certificate_with_capacity(
                "c1", 0, None, 1, 2
            ))),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(
            f.apply(FrontierEvent::SetCapacity {
                listing_hash: hash(3),
                listing_revision: hash(7),
                capacity: 2,
                seller: agent(2),
                sequence: 1,
                previous_certificate_id: Some("c1".into()),
            }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(
            f.apply(FrontierEvent::Release {
                certificate_id: "c1".into(),
                seller: agent(2),
                sequence: 2,
                previous_certificate_id: Some("capacity:1".into()),
            }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(f.active_reserved(), 0);
        assert_eq!(f.available(), 2);
    }

    #[test]
    fn terminal_replay_after_listing_revision_bridge_is_idempotent() {
        let mut f = frontier(2);
        f.apply(FrontierEvent::Reserve(certificate_with_capacity(
            "c1", 0, None, 1, 2,
        )))
        .unwrap();
        f.apply(FrontierEvent::SetCapacity {
            listing_hash: hash(3),
            listing_revision: hash(7),
            capacity: 2,
            seller: agent(2),
            sequence: 1,
            previous_certificate_id: Some("c1".into()),
        })
        .unwrap();

        let terminal = FrontierEvent::Release {
            certificate_id: "c1".into(),
            seller: agent(2),
            sequence: 2,
            previous_certificate_id: Some("capacity:1".into()),
        };
        assert_eq!(f.apply(terminal.clone()), Ok(ApplyOutcome::Applied));
        assert_eq!(f.apply(terminal), Ok(ApplyOutcome::Idempotent));
    }

    #[test]
    fn historical_consume_is_valid_after_listing_revision_bridge() {
        let mut f = frontier(2);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(certificate_with_capacity(
                "c1", 0, None, 1, 2
            ))),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(
            f.apply(FrontierEvent::SetCapacity {
                listing_hash: hash(3),
                listing_revision: hash(7),
                capacity: 2,
                seller: agent(2),
                sequence: 1,
                previous_certificate_id: Some("c1".into()),
            }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(
            f.apply(FrontierEvent::Consume {
                certificate_id: "c1".into(),
                seller: agent(2),
                sequence: 2,
                previous_certificate_id: Some("capacity:1".into()),
            }),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(f.active_reserved(), 0);
        assert_eq!(f.capacity(), 1);
        assert_eq!(f.available(), 1);
    }

    #[test]
    fn consume_permanently_removes_capacity() {
        let mut f = frontier(1);
        f.apply(FrontierEvent::Reserve(certificate_with_capacity(
            "c1", 0, None, 1, 1,
        )))
        .unwrap();
        f.apply(FrontierEvent::Consume {
            certificate_id: "c1".into(),
            seller: agent(2),
            sequence: 1,
            previous_certificate_id: Some("c1".into()),
        })
        .unwrap();
        assert_eq!(f.capacity(), 0);
        assert_eq!(f.available(), 0);
    }

    #[test]
    fn forked_frontier_is_rejected_by_previous_certificate_binding() {
        let mut f = frontier(3);
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 1)))
            .unwrap();
        let error = f
            .apply(FrontierEvent::Reserve(certificate(
                "c2",
                1,
                Some("other-head"),
                1,
            )))
            .unwrap_err();
        assert!(matches!(error, CertificateError::PreviousMismatch { .. }));
    }

    #[test]
    fn stale_listing_revision_cannot_admit_new_reservation() {
        let mut f = frontier(2);
        let mut stale = certificate("c1", 0, None, 1);
        stale.listing_revision = hash(9);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(stale)),
            Err(CertificateError::ListingRevisionMismatch)
        );
    }

    #[test]
    fn buyer_fields_are_bound_to_the_intent_and_seller() {
        let mut f = frontier(2);
        let mut forged = certificate("c1", 0, None, 1);
        forged.intent.buyer = agent(7);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(forged)),
            Ok(ApplyOutcome::Applied)
        );
        assert_eq!(f.seller(), &agent(2));
    }

    #[test]
    fn terminal_conflict_cannot_cross_release_and_consume() {
        let mut f = frontier(1);
        f.apply(FrontierEvent::Reserve(certificate_with_capacity(
            "c1", 0, None, 1, 1,
        )))
        .unwrap();
        f.apply(FrontierEvent::Release {
            certificate_id: "c1".into(),
            seller: agent(2),
            sequence: 1,
            previous_certificate_id: Some("c1".into()),
        })
        .unwrap();
        let error = f
            .apply(FrontierEvent::Consume {
                certificate_id: "c1".into(),
                seller: agent(2),
                sequence: 2,
                previous_certificate_id: Some("c1:release".into()),
            })
            .unwrap_err();
        assert!(matches!(error, CertificateError::AlreadyReleased(_)));
    }

    #[test]
    fn capacity_state_evidence_is_exactly_foldable() {
        let pre = ReservationFrontierState {
            capacity: 10,
            active_reserved: 3,
            available: 7,
        };
        let evidence = ReservationCapacityEvidence {
            seller: agent(2),
            listing_hash: hash(3),
            listing_revision: hash(4),
            capacity: 12,
            sequence: 1,
            previous_frontier_action: hash(9),
            pre_state: pre.clone(),
            post_state: pre.after_set_capacity(12).unwrap(),
        };
        assert!(evidence.validate_state_transition().is_ok());
    }

    #[test]
    fn capacity_state_evidence_rejects_capacity_below_reservations() {
        let pre = ReservationFrontierState {
            capacity: 10,
            active_reserved: 3,
            available: 7,
        };
        let evidence = ReservationCapacityEvidence {
            seller: agent(2),
            listing_hash: hash(3),
            listing_revision: hash(4),
            capacity: 2,
            sequence: 1,
            previous_frontier_action: hash(9),
            pre_state: pre,
            post_state: ReservationFrontierState {
                capacity: 2,
                active_reserved: 3,
                available: 0,
            },
        };
        assert!(evidence.validate_state_transition().is_err());
    }

    #[test]
    fn terminal_state_evidence_is_exactly_foldable() {
        let certificate = certificate_with_capacity("c1", 0, None, 1, 2);
        let released = ReservationTerminalEvidence {
            certificate_hash: hash(9),
            seller: agent(2),
            intent_hash: certificate.intent_hash.clone(),
            outcome: ReservationTerminalOutcome::Released,
            sequence: 1,
            previous_frontier_action: hash(9),
            pre_state: certificate.post_state.clone(),
            post_state: certificate.post_state.after_release(1).unwrap(),
        };
        assert!(released.validate_state_transition(&certificate).is_ok());

        let consumed = ReservationTerminalEvidence {
            outcome: ReservationTerminalOutcome::Consumed,
            post_state: certificate.post_state.after_consume(1).unwrap(),
            ..released.clone()
        };
        assert!(consumed.validate_state_transition(&certificate).is_ok());
    }

    #[test]
    fn terminal_state_evidence_rejects_forged_post_state() {
        let certificate = certificate_with_capacity("c1", 0, None, 1, 2);
        let evidence = ReservationTerminalEvidence {
            certificate_hash: hash(9),
            seller: agent(2),
            intent_hash: certificate.intent_hash.clone(),
            outcome: ReservationTerminalOutcome::Released,
            sequence: 1,
            previous_frontier_action: hash(9),
            pre_state: certificate.post_state.clone(),
            post_state: ReservationFrontierState {
                capacity: 2,
                active_reserved: 0,
                available: 1,
            },
        };
        assert!(evidence.validate_state_transition(&certificate).is_err());
    }

    fn transaction_for(intent: &PurchaseIntent) -> crate::Transaction {
        crate::Transaction {
            buyer: intent.buyer.clone(),
            seller: intent.seller.clone(),
            listing_hash: intent.listing_hash.clone(),
            reservation_certificate_hash: hash(99),
            quantity: intent.quantity,
            total_price_cents: intent.unit_price_cents * u64::from(intent.quantity),
            status: crate::TransactionStatus::Pending,
            created_at: Timestamp::from_micros(1_000_000),
            updated_at: Timestamp::from_micros(1_000_000),
            tracking_info: None,
            epistemic: crate::EpistemicClassification {
                empirical: crate::EmpiricalLevel::E1Testimonial,
                normative: crate::NormativeLevel::N1Communal,
                materiality: crate::MaterialityLevel::M1Temporal,
            },
        }
    }

    #[test]
    fn transaction_binding_accepts_exact_certificate_terms() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let tx = transaction_for(&i);
        assert!(validate_transaction_reservation_binding(&tx, &c).is_ok());
    }

    #[test]
    fn transaction_binding_rejects_cross_listing_certificate() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.listing_hash = hash(77);
        assert!(
            validate_transaction_reservation_binding(&tx, &c)
                .unwrap_err()
                .contains("listing")
        );
    }

    #[test]
    fn transaction_binding_rejects_cross_buyer_certificate() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.buyer = agent(88);
        assert!(
            validate_transaction_reservation_binding(&tx, &c)
                .unwrap_err()
                .contains("buyer")
        );
    }

    #[test]
    fn transaction_binding_rejects_quantity_mismatch() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.quantity = 2;
        tx.total_price_cents = i.unit_price_cents * 2;
        assert!(
            validate_transaction_reservation_binding(&tx, &c)
                .unwrap_err()
                .contains("quantity")
        );
    }

    #[test]
    fn transaction_binding_rejects_price_mismatch() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.total_price_cents += 1;
        assert!(
            validate_transaction_reservation_binding(&tx, &c)
                .unwrap_err()
                .contains("total")
        );
    }

    #[test]
    fn transaction_binding_rejects_certificate_that_is_listing_hash() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.reservation_certificate_hash = tx.listing_hash.clone();
        assert!(
            validate_transaction_reservation_binding(&tx, &c)
                .unwrap_err()
                .contains("distinct")
        );
    }
}

/// Immutable seller-authored terminal evidence for one exact reservation.
#[hdk_entry_helper]
#[derive(Debug, Clone, PartialEq)]
pub struct ReservationTerminalEvidence {
    pub certificate_hash: ActionHash,
    pub seller: AgentPubKey,
    pub intent_hash: ActionHash,
    pub outcome: ReservationTerminalOutcome,
    pub sequence: u64,
    pub previous_frontier_action: ActionHash,
    /// Economic state immediately before this terminal transition.
    pub pre_state: ReservationFrontierState,
    /// Economic state immediately after this terminal transition.
    pub post_state: ReservationFrontierState,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ReservationTerminalOutcome {
    Released,
    Consumed,
}

/// Immutable seller-authored capacity revision evidence.
#[hdk_entry_helper]
#[derive(Debug, Clone, PartialEq)]
pub struct ReservationCapacityEvidence {
    pub seller: AgentPubKey,
    pub listing_hash: ActionHash,
    pub listing_revision: ActionHash,
    pub capacity: u32,
    pub sequence: u64,
    pub previous_frontier_action: ActionHash,
    pub pre_state: ReservationFrontierState,
    pub post_state: ReservationFrontierState,
}

impl ReservationCapacityEvidence {
    pub fn validate_state_transition(&self) -> Result<(), &'static str> {
        self.pre_state.validate()?;
        self.post_state.validate()?;
        self.pre_state.validate_transition(
            FrontierStateTransition::SetCapacity {
                capacity: self.capacity,
            },
            &self.post_state,
        )
    }
}

impl ReservationTerminalEvidence {
    /// Pure validation of the economic state transition represented by this
    /// terminal event. The certificate is the immutable economic reservation
    /// being terminated; the frontier predecessor is supplied separately by
    /// the Holochain validation boundary and may be a later capacity/revision
    /// bridge.
    pub fn validate_state_transition(
        &self,
        certificate: &ReservationCertificate,
    ) -> Result<(), &'static str> {
        if self.seller != certificate.seller {
            return Err("Terminal evidence seller does not match certificate seller");
        }
        if self.intent_hash != certificate.intent_hash {
            return Err("Terminal evidence intent does not match certificate intent");
        }

        let transition = match self.outcome {
            ReservationTerminalOutcome::Released => FrontierStateTransition::Release {
                quantity: certificate.quantity,
            },
            ReservationTerminalOutcome::Consumed => FrontierStateTransition::Consume {
                quantity: certificate.quantity,
            },
        };
        self.pre_state
            .validate_transition(transition, &self.post_state)
    }
}

/// Purely validate that a transaction is authorized by one exact reservation certificate.
///
/// This is deliberately independent of DHT access so coordinator/integrity tests can
/// exercise the economic binding without constructing a full Holochain validation context.
pub fn validate_transaction_reservation_binding(
    transaction: &crate::Transaction,
    certificate: &ReservationCertificate,
) -> Result<(), String> {
    certificate
        .validate()
        .map_err(|error| format!("Invalid reservation certificate: {error:?}"))?;

    if transaction.reservation_certificate_hash == transaction.listing_hash {
        return Err("Reservation certificate must be distinct from the listing hash".into());
    }
    if certificate.seller != transaction.seller {
        return Err("Transaction seller does not match reservation certificate seller".into());
    }
    if certificate.intent.buyer != transaction.buyer {
        return Err("Transaction buyer does not match reservation intent buyer".into());
    }
    if certificate.listing_hash != transaction.listing_hash {
        return Err("Transaction listing does not match reservation certificate listing".into());
    }
    if certificate.quantity != transaction.quantity {
        return Err("Transaction quantity does not match reservation certificate quantity".into());
    }
    let expected_total = certificate
        .intent
        .unit_price_cents
        .checked_mul(u64::from(transaction.quantity))
        .ok_or_else(|| "Reservation transaction total overflow".to_string())?;
    if transaction.total_price_cents != expected_total {
        return Err("Transaction total does not match reservation terms".into());
    }

    Ok(())
}

/// Validate a buyer-authored PurchaseIntent at the Holochain boundary.
pub fn validate_create_purchase_intent(
    intent: &PurchaseIntent,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = intent.validate() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid purchase intent: {error:?}"
        )));
    }
    if action.author != intent.buyer {
        return Ok(ValidateCallbackResult::Invalid(
            "PurchaseIntent must be authored by its buyer".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Prove that a listing revision is the seller's latest root-derived revision
/// before the action currently being validated.
///
/// Listing updates may be interleaved with unrelated seller-chain actions, so
/// source-chain adjacency is deliberately not required. Instead, the bounded
/// source-chain slice is searched for the listing root and every Update whose
/// original_action_address is that root; the greatest action sequence is the
/// authoritative current revision. This is deterministic because action
/// sequence is part of the source-chain action and hash-bounded activity is a
/// contiguous source-chain slice.
fn validate_current_listing_revision(
    seller: &AgentPubKey,
    listing_hash: &ActionHash,
    candidate_revision: &ActionHash,
    chain_top: &ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let activity = must_get_agent_activity(
        seller.clone(),
        ChainFilter::new(chain_top.clone()).until_hash(listing_hash.clone()),
    )?;

    let mut latest_revision: Option<(u32, ActionHash)> = None;
    let mut candidate_seq = None;
    let mut root_seen = false;

    for item in activity {
        let action = item.action.hashed.content;
        let action_hash = item.action.hashed.hash;
        let sequence = action.action_seq();

        if action_hash == *candidate_revision {
            candidate_seq = Some(sequence);
        }

        match action {
            Action::Create(_) if action_hash == *listing_hash => {
                root_seen = true;
                if latest_revision
                    .as_ref()
                    .map(|(seq, _)| sequence > *seq)
                    .unwrap_or(true)
                {
                    latest_revision = Some((sequence, action_hash));
                }
            }
            Action::Update(update) if update.original_action_address == *listing_hash => {
                if latest_revision
                    .as_ref()
                    .map(|(seq, _)| sequence > *seq)
                    .unwrap_or(true)
                {
                    latest_revision = Some((sequence, action_hash));
                }
            }
            _ => {}
        }
    }

    if !root_seen {
        return Ok(ValidateCallbackResult::Invalid(
            "Listing root is not present in the seller source-chain ancestry".into(),
        ));
    }

    let Some(candidate_seq) = candidate_seq else {
        return Ok(ValidateCallbackResult::Invalid(
            "Listing revision is not present in the seller source-chain ancestry".into(),
        ));
    };
    let Some((latest_seq, latest_revision)) = latest_revision else {
        return Ok(ValidateCallbackResult::Invalid(
            "Listing has no root-derived revision in the seller source chain".into(),
        ));
    };

    if latest_revision != *candidate_revision || latest_seq != candidate_seq {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation evidence references a stale listing revision".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a seller-issued reservation against exact addressable dependencies.
/// This intentionally avoids mutable link collections in validation.
pub fn validate_create_reservation_certificate(
    certificate: &ReservationCertificate,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = certificate.validate() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid reservation certificate: {error:?}"
        )));
    }
    if action.author != certificate.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate must be authored by its seller".into(),
        ));
    }

    let intent_record = must_get_valid_record(certificate.intent_hash.clone())?;
    let intent = intent_record
        .entry()
        .to_app_option::<PurchaseIntent>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid PurchaseIntent entry: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "ReservationCertificate intent dependency has the wrong entry type".into(),
            ))
        })?;

    if intent != certificate.intent
        || intent.seller != certificate.seller
        || intent.listing_hash != certificate.listing_hash
        || intent.listing_revision != certificate.listing_revision
        || intent.quantity != certificate.quantity
    {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate does not exactly bind its PurchaseIntent".into(),
        ));
    }

    // The action hash must resolve to an actual, already-valid Marketplace Listing;
    // an arbitrary seller-authored Create/Update action is not sufficient authority.
    let listing_record = must_get_valid_record(certificate.listing_hash.clone())?;
    let listing = listing_record
        .entry()
        .to_app_option::<Listing>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid listing entry: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "ReservationCertificate listing dependency is not a Listing entry".into(),
            ))
        })?;
    let listing_action = listing_record.action();
    if listing_action.author() != &certificate.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate seller does not own the referenced listing action".into(),
        ));
    }

    // Keep the decoded Listing live so a successful type check is an explicit
    // dependency of validation; the transaction protocol currently does not
    // require a particular ListingStatus here because inventory authority is
    // represented by the reservation frontier itself.
    let _ = listing;

    let revision_record = must_get_valid_record(certificate.listing_revision.clone())?;
    let revision_action = must_get_action(certificate.listing_revision.clone())?;
    let _revision_listing = revision_record
        .entry()
        .to_app_option::<Listing>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid listing revision entry: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "ReservationCertificate listing revision is not a Listing entry".into(),
            ))
        })?;
    if revision_action.author() != &certificate.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate listing revision is not seller-authored".into(),
        ));
    }

    if _revision_listing.price_cents != certificate.intent.unit_price_cents {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate unit price does not match the bound listing revision".into(),
        ));
    }
    if certificate.sequence == 0
        && certificate.pre_state.capacity != _revision_listing.quantity_available
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Genesis reservation capacity does not match the bound listing revision inventory".into(),
        ));
    }

    match revision_action.action() {
        Action::Create(_) => {
            if certificate.listing_revision != certificate.listing_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "A create action can only be the listing's root revision".into(),
                ));
            }
        }
        Action::Update(update) => {
            if update.original_action_address != certificate.listing_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "Listing revision does not descend from the referenced listing root".into(),
                ));
            }
        }
        _ => {
            return Ok(ValidateCallbackResult::Invalid(
                "Listing revision must reference a listing create or update action".into(),
            ));
        }
    }
    if let Some(chain_top) = action.prev_action.clone() {
        let revision_result = validate_current_listing_revision(
            &certificate.seller,
            &certificate.listing_hash,
            &certificate.listing_revision,
            &chain_top,
        )?;
        if !matches!(revision_result, ValidateCallbackResult::Valid) {
            return Ok(revision_result);
        }
    } else {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate action is missing its seller source-chain predecessor".into(),
        ));
    }

    if certificate.sequence > 0 {
        let previous_hash = certificate
            .previous_frontier_action
            .clone()
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Non-genesis reservation certificate omitted previous frontier action".into(),
                ))
            })?;

        // The economic frontier predecessor must be an earlier action on the
        // same seller source chain, but it need not be the immediately preceding
        // source-chain action. Sellers can legitimately interleave unrelated
        // Marketplace records (messages, listings, updates, etc.) between
        // inventory events. Holochain's source-chain continuity plus this bounded
        // activity proof prevents a forked/later action from being smuggled in as
        // the predecessor without imposing global adjacency on the seller chain.
        let prior_activity = must_get_agent_activity(
            certificate.seller.clone(),
            ChainFilter::new(action.prev_action.clone()).until_hash(previous_hash.clone()),
        )?;
        if !prior_activity
            .iter()
            .any(|activity| activity.action.hashed.hash == previous_hash)
        {
            return Ok(ValidateCallbackResult::Invalid(
                "Reservation frontier predecessor is not an earlier action on the seller source chain".into(),
            ));
        }

        let previous = must_get_valid_record(previous_hash)?;
        if previous.action().author() != &certificate.seller {
            return Ok(ValidateCallbackResult::Invalid(
                "Previous frontier record is not seller-authored".into(),
            ));
        }

        // The frontier is per seller + listing root. Listing revisions may
        // change over time, but each new reservation must bind to the current
        // revision proved above. Existing terminal events remain bound to the
        // exact historical certificate terms.
        if let Some(previous_certificate) = previous
            .entry()
            .to_app_option::<ReservationCertificate>()
            .map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid previous frontier certificate entry: {e:?}"
                )))
            })?
        {
            if previous_certificate.seller != certificate.seller
                || previous_certificate.listing_hash != certificate.listing_hash
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Previous frontier certificate belongs to a different seller/listing/revision"
                        .into(),
                ));
            }
            if previous_certificate.sequence.checked_add(1) != Some(certificate.sequence) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Reservation frontier sequence does not follow its previous certificate".into(),
                ));
            }
            if previous_certificate.post_state != certificate.pre_state {
                return Ok(ValidateCallbackResult::Invalid(
                    "Reservation frontier post-state does not equal the next reservation pre-state"
                        .into(),
                ));
            }
        } else if let Some(previous_terminal) = previous
            .entry()
            .to_app_option::<ReservationTerminalEvidence>()
            .map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid previous frontier terminal entry: {e:?}"
                )))
            })?
        {
            if previous_terminal.seller != certificate.seller {
                return Ok(ValidateCallbackResult::Invalid(
                    "Previous frontier terminal evidence belongs to another seller".into(),
                ));
            }
            let terminal_certificate =
                must_get_valid_record(previous_terminal.certificate_hash.clone())?;
            let terminal_certificate = terminal_certificate
                .entry()
                .to_app_option::<ReservationCertificate>()
                .map_err(|e| {
                    wasm_error!(WasmErrorInner::Guest(format!(
                        "Invalid previous frontier terminal certificate: {e:?}"
                    )))
                })?
                .ok_or_else(|| {
                    wasm_error!(WasmErrorInner::Guest(
                        "Previous frontier terminal certificate has the wrong entry type".into(),
                    ))
                })?;
            if terminal_certificate.seller != certificate.seller
                || terminal_certificate.listing_hash != certificate.listing_hash
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Previous frontier terminal evidence belongs to a different seller/listing/revision".into(),
                ));
            }
            if previous_terminal.sequence.checked_add(1) != Some(certificate.sequence) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Reservation frontier sequence does not follow its previous terminal event"
                        .into(),
                ));
            }
            if previous_terminal.post_state != certificate.pre_state {
                return Ok(ValidateCallbackResult::Invalid(
                    "Reservation frontier terminal post-state does not equal the next reservation pre-state".into(),
                ));
            }
        } else {
            return Ok(ValidateCallbackResult::Invalid(
                "Previous frontier action is not a recognized reservation frontier event".into(),
            ));
        }
    } else if certificate.previous_frontier_action.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Genesis reservation certificate cannot reference a previous frontier".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate seller-authored capacity revision evidence and bind it to the exact
/// listing revision and preceding frontier state.
pub fn validate_create_reservation_capacity(
    evidence: &ReservationCapacityEvidence,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if action.author != evidence.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity evidence must be authored by the seller".into(),
        ));
    }
    if evidence.sequence == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity evidence cannot be a genesis event".into(),
        ));
    }
    if let Err(error) = evidence.validate_state_transition() {
        return Ok(ValidateCallbackResult::Invalid(error.into()));
    }

    let listing_action = must_get_action(evidence.listing_hash.clone())?;
    if listing_action.author() != &evidence.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity listing is not seller-authored".into(),
        ));
    }
    let revision_record = must_get_valid_record(evidence.listing_revision.clone())?;
    let revision_listing = revision_record
        .entry()
        .to_app_option::<Listing>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid reservation capacity listing revision entry: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Reservation capacity listing revision is not a Listing entry".into(),
            ))
        })?;
    let revision_action = must_get_action(evidence.listing_revision.clone())?;
    if revision_action.author() != &evidence.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity listing revision is not seller-authored".into(),
        ));
    }
    if evidence.capacity != revision_listing.quantity_available {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity does not match the bound listing revision inventory".into(),
        ));
    }
    match revision_action.action() {
        Action::Create(_) if evidence.listing_revision == evidence.listing_hash => {}
        Action::Update(update) if update.original_action_address == evidence.listing_hash => {}
        _ => {
            return Ok(ValidateCallbackResult::Invalid(
                "Reservation capacity evidence is not bound to a valid listing revision".into(),
            ));
        }
    }
    if let Some(chain_top) = action.prev_action.clone() {
        let revision_result = validate_current_listing_revision(
            &evidence.seller,
            &evidence.listing_hash,
            &evidence.listing_revision,
            &chain_top,
        )?;
        if !matches!(revision_result, ValidateCallbackResult::Valid) {
            return Ok(revision_result);
        }
    } else {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity evidence action is missing its seller source-chain predecessor"
                .into(),
        ));
    }

    let prior_activity = must_get_agent_activity(
        evidence.seller.clone(),
        ChainFilter::new(action.prev_action.clone())
            .until_hash(evidence.previous_frontier_action.clone()),
    )?;
    if !prior_activity
        .iter()
        .any(|activity| activity.action.hashed.hash == evidence.previous_frontier_action)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity predecessor is not an earlier action on the seller source chain"
                .into(),
        ));
    }

    let previous = must_get_valid_record(evidence.previous_frontier_action.clone())?;
    if previous.action().author() != &evidence.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity predecessor is not seller-authored".into(),
        ));
    }

    let predecessor_state = if let Some(certificate) = previous
        .entry()
        .to_app_option::<ReservationCertificate>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid reservation certificate: {e:?}"
            )))
        })? {
        if certificate.seller != evidence.seller
            || certificate.listing_hash != evidence.listing_hash
            || certificate.sequence.checked_add(1) != Some(evidence.sequence)
        {
            return Ok(ValidateCallbackResult::Invalid(
                "Reservation capacity predecessor does not match the frontier domain or sequence"
                    .into(),
            ));
        }
        certificate.post_state
    } else if let Some(terminal) = previous
        .entry()
        .to_app_option::<ReservationTerminalEvidence>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid terminal evidence: {e:?}"
            )))
        })?
    {
        let certificate_record = must_get_valid_record(terminal.certificate_hash.clone())?;
        let certificate = certificate_record
            .entry()
            .to_app_option::<ReservationCertificate>()
            .map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid terminal certificate: {e:?}"
                )))
            })?
            .ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "Terminal predecessor has wrong certificate type".into()
                ))
            })?;
        if certificate.seller != evidence.seller
            || certificate.listing_hash != evidence.listing_hash
            || terminal.sequence.checked_add(1) != Some(evidence.sequence)
        {
            return Ok(ValidateCallbackResult::Invalid(
                "Reservation capacity terminal predecessor does not match the frontier domain or sequence".into(),
            ));
        }
        terminal.post_state
    } else if let Some(capacity) = previous
        .entry()
        .to_app_option::<ReservationCapacityEvidence>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid capacity evidence: {e:?}"
            )))
        })?
    {
        if capacity.seller != evidence.seller
            || capacity.listing_hash != evidence.listing_hash
            || capacity.sequence.checked_add(1) != Some(evidence.sequence)
        {
            return Ok(ValidateCallbackResult::Invalid(
                "Reservation capacity predecessor does not match the frontier domain or sequence"
                    .into(),
            ));
        }
        capacity.post_state
    } else {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity predecessor is not a recognized frontier event".into(),
        ));
    };

    // The predecessor may belong to the previous listing revision: this
    // evidence is the explicit frontier bridge to the newly current revision.
    if predecessor_state != evidence.pre_state {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation capacity pre-state does not equal predecessor post-state".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate seller-authored terminal evidence and bind it to the exact
/// reservation certificate and immediately preceding frontier action.
pub fn validate_create_reservation_terminal(
    evidence: &ReservationTerminalEvidence,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if action.author != evidence.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal evidence must be authored by the seller".into(),
        ));
    }

    let certificate_record = must_get_valid_record(evidence.certificate_hash.clone())?;
    let certificate = certificate_record
        .entry()
        .to_app_option::<ReservationCertificate>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid certificate entry: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Reservation terminal certificate dependency has the wrong entry type".into(),
            ))
        })?;

    if certificate.seller != evidence.seller || certificate.intent_hash != evidence.intent_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal evidence does not bind to the certificate seller/intent".into(),
        ));
    }

    if let Err(error) = evidence.validate_state_transition(&certificate) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Reservation terminal state transition is invalid: {error}"
        )));
    }

    let Some(chain_top) = action.prev_action.clone() else {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal evidence action is missing its seller source-chain predecessor"
                .into(),
        ));
    };

    // The certificate is the historical reservation being terminated. It may
    // be separated from this terminal event by later seller-authored frontier
    // events (for example a listing-revision/capacity bridge). Therefore the
    // certificate remains an economic dependency, while the actual frontier
    // predecessor determines the terminal sequence and pre-state.
    let prior_activity = must_get_agent_activity(
        evidence.seller.clone(),
        ChainFilter::new(chain_top.clone()).until_hash(evidence.certificate_hash.clone()),
    )?;
    if !prior_activity
        .iter()
        .any(|activity| activity.action.hashed.hash == evidence.certificate_hash)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal certificate is not an earlier action on the seller source chain"
                .into(),
        ));
    }
    if !prior_activity
        .iter()
        .any(|activity| activity.action.hashed.hash == evidence.previous_frontier_action)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal frontier predecessor is not an earlier action on the seller source chain".into(),
        ));
    }

    let previous = must_get_valid_record(evidence.previous_frontier_action.clone())?;
    if previous.action().author() != &evidence.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal previous frontier is not seller-authored".into(),
        ));
    }

    let (previous_sequence, previous_post_state, previous_listing_hash) =
        if let Some(previous_certificate) = previous
            .entry()
            .to_app_option::<ReservationCertificate>()
            .map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid previous frontier certificate entry: {e:?}"
                )))
            })?
        {
            (
                previous_certificate.sequence,
                previous_certificate.post_state,
                previous_certificate.listing_hash,
            )
        } else if let Some(previous_terminal) = previous
            .entry()
            .to_app_option::<ReservationTerminalEvidence>()
            .map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid previous frontier terminal entry: {e:?}"
                )))
            })?
        {
            let previous_certificate =
                must_get_valid_record(previous_terminal.certificate_hash.clone())?
                    .entry()
                    .to_app_option::<ReservationCertificate>()
                    .map_err(|e| {
                        wasm_error!(WasmErrorInner::Guest(format!(
                            "Invalid previous terminal certificate entry: {e:?}"
                        )))
                    })?
                    .ok_or_else(|| {
                        wasm_error!(WasmErrorInner::Guest(
                            "Previous terminal certificate has the wrong entry type".into(),
                        ))
                    })?;

            (
                previous_terminal.sequence,
                previous_terminal.post_state,
                previous_certificate.listing_hash,
            )
        } else if let Some(previous_capacity) = previous
            .entry()
            .to_app_option::<ReservationCapacityEvidence>()
            .map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid previous capacity entry: {e:?}"
                )))
            })?
        {
            (
                previous_capacity.sequence,
                previous_capacity.post_state,
                previous_capacity.listing_hash,
            )
        } else {
            return Ok(ValidateCallbackResult::Invalid(
                "Reservation terminal previous frontier is not a recognized frontier event".into(),
            ));
        };

    if previous_listing_hash != certificate.listing_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal frontier predecessor belongs to a different listing".into(),
        ));
    }
    if previous_sequence < certificate.sequence {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal frontier predecessor predates the historical reservation".into(),
        ));
    }
    if previous_sequence == certificate.sequence
        && evidence.previous_frontier_action != evidence.certificate_hash
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal predecessor at the reservation sequence must be the certificate itself"
                .into(),
        ));
    }
    if previous_sequence.checked_add(1) != Some(evidence.sequence) {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal evidence sequence does not follow its actual frontier predecessor".into(),
        ));
    }
    if previous_post_state != evidence.pre_state {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal pre-state does not equal its actual frontier predecessor post-state".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

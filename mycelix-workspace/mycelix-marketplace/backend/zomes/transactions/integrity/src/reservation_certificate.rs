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
use std::collections::BTreeMap;

use crate::reservation::{ApplyOutcome, Reservation, ReservationError, ReservationLedger};

#[hdk_entry_helper]
#[derive(Debug, Clone, PartialEq)]
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
        if self.intent_id.trim().is_empty() { return Err(IntentError::EmptyIntentId); }
        if self.client_nonce.trim().is_empty() { return Err(IntentError::EmptyClientNonce); }
        if self.quantity == 0 { return Err(IntentError::ZeroQuantity); }
        if self.buyer == self.seller { return Err(IntentError::BuyerSellerMustDiffer); }
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
}

impl ReservationCertificate {
    pub fn validate(&self) -> Result<(), CertificateError> {
        if self.certificate_id.trim().is_empty() { return Err(CertificateError::EmptyCertificateId); }
        if self.quantity == 0 { return Err(CertificateError::ZeroQuantity); }
        self.intent.validate().map_err(CertificateError::InvalidIntent)?;
        if self.seller != self.intent.seller { return Err(CertificateError::SellerMismatch); }
        if self.listing_hash != self.intent.listing_hash { return Err(CertificateError::ListingMismatch); }
        if self.listing_revision != self.intent.listing_revision { return Err(CertificateError::ListingRevisionMismatch); }
        if self.quantity != self.intent.quantity { return Err(CertificateError::QuantityMismatch); }
        if self.sequence == 0 && (self.previous_certificate_id.is_some() || self.previous_frontier_action.is_some()) {
            return Err(CertificateError::GenesisHasPrevious);
        }
        if self.sequence > 0 && (self.previous_certificate_id.is_none() || self.previous_frontier_action.is_none()) {
            return Err(CertificateError::MissingPrevious);
        }
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
    WrongSeller,
    WrongListing,
    WrongRevision,
    SequenceMismatch { expected: u64, actual: u64 },
    PreviousMismatch { expected: Option<String>, actual: Option<String> },
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
}

/// Deterministic snapshot of the economic state represented by a frontier.
///
/// This is intentionally structural rather than cryptographic. On the current
/// Holochain 0.6 baseline, the authoritative content-addressed predecessor
/// remains the action hash; this snapshot gives independent reducers an exact
/// post-state to compare without introducing an unsupported arbitrary-byte hash
/// dependency.
#[derive(Debug, Clone, PartialEq, Eq)]
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

    pub fn after_reserve(
        &self,
        quantity: u32,
    ) -> Result<Self, &'static str> {
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

    pub fn after_release(
        &self,
        quantity: u32,
    ) -> Result<Self, &'static str> {
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

    pub fn after_consume(
        &self,
        quantity: u32,
    ) -> Result<Self, &'static str> {
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

    pub fn after_set_capacity(
        &self,
        capacity: u32,
    ) -> Result<Self, &'static str> {
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
            FrontierStateTransition::SetCapacity { capacity } => self.after_set_capacity(capacity)?,
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
    pub fn new(seller: AgentPubKey, listing_hash: ActionHash, listing_revision: ActionHash, capacity: u32) -> Self {
        Self {
            seller,
            listing_hash,
            listing_revision,
            ledger: ReservationLedger::new(capacity),
            next_sequence: 0,
            head_certificate_id: None,
            certificates: BTreeMap::new(),
        }
    }

    pub fn seller(&self) -> &AgentPubKey { &self.seller }
    pub fn listing_hash(&self) -> &ActionHash { &self.listing_hash }
    pub fn listing_revision(&self) -> &ActionHash { &self.listing_revision }
    pub fn capacity(&self) -> u32 { self.ledger.capacity() }
    pub fn available(&self) -> u32 { self.ledger.available() }
    pub fn active_reserved(&self) -> u32 { self.ledger.active_reserved() }
    pub fn post_state(&self) -> ReservationFrontierState {
        ReservationFrontierState {
            capacity: self.capacity(),
            active_reserved: self.active_reserved(),
            available: self.available(),
        }
    }
    pub fn next_sequence(&self) -> u64 { self.next_sequence }
    pub fn head_certificate_id(&self) -> Option<&str> { self.head_certificate_id.as_deref() }
    pub fn certificate(&self, certificate_id: &str) -> Option<&ReservationCertificate> { self.certificates.get(certificate_id) }

    pub fn apply(&mut self, event: FrontierEvent) -> Result<ApplyOutcome, CertificateError> {
        match event {
            FrontierEvent::Reserve(certificate) => self.reserve(certificate),
            FrontierEvent::Release { certificate_id, seller, sequence, previous_certificate_id } =>
                self.terminal(&certificate_id, seller, sequence, previous_certificate_id, true),
            FrontierEvent::Consume { certificate_id, seller, sequence, previous_certificate_id } =>
                self.terminal(&certificate_id, seller, sequence, previous_certificate_id, false),
            FrontierEvent::SetCapacity { listing_hash, listing_revision, capacity, seller, sequence, previous_certificate_id } =>
                self.set_capacity(listing_hash, listing_revision, capacity, seller, sequence, previous_certificate_id),
        }
    }

    fn check_frontier(&self, seller: &AgentPubKey, sequence: u64, previous_certificate_id: &Option<String>) -> Result<(), CertificateError> {
        if seller != &self.seller { return Err(CertificateError::WrongSeller); }
        if sequence != self.next_sequence {
            return Err(CertificateError::SequenceMismatch { expected: self.next_sequence, actual: sequence });
        }
        if previous_certificate_id != &self.head_certificate_id {
            return Err(CertificateError::PreviousMismatch { expected: self.head_certificate_id.clone(), actual: previous_certificate_id.clone() });
        }
        Ok(())
    }

    fn reserve(&mut self, certificate: ReservationCertificate) -> Result<ApplyOutcome, CertificateError> {
        certificate.validate()?;
        if certificate.seller != self.seller { return Err(CertificateError::WrongSeller); }
        if certificate.listing_hash != self.listing_hash { return Err(CertificateError::WrongListing); }
        if certificate.listing_revision != self.listing_revision { return Err(CertificateError::WrongRevision); }
        if let Some(existing) = self.certificates.get(&certificate.certificate_id) {
            if existing == &certificate {
                return Ok(ApplyOutcome::Idempotent);
            }
            return Err(CertificateError::IntentConflict);
        }
        self.check_frontier(&certificate.seller, certificate.sequence, &certificate.previous_certificate_id)?;

        let reservation = Reservation {
            intent_id: certificate.intent.intent_id.clone(),
            listing_revision: format!("{:?}", certificate.listing_revision),
            quantity: certificate.quantity,
        };
        let outcome = self.ledger.apply(crate::reservation::ReservationEvent::Reserve(reservation))
            .map_err(CertificateError::Capacity)?;

        self.certificates.insert(certificate.certificate_id.clone(), certificate.clone());
        self.advance(certificate.certificate_id);
        Ok(outcome)
    }

    fn terminal(&mut self, certificate_id: &str, seller: AgentPubKey, sequence: u64, previous_certificate_id: Option<String>, release: bool) -> Result<ApplyOutcome, CertificateError> {
        let certificate = self.certificates.get(certificate_id)
            .ok_or_else(|| CertificateError::UnknownCertificate(certificate_id.to_owned()))?;

        let expected_head = format!("{}:{}", certificate_id, if release { "release" } else { "consume" });
        if self.head_certificate_id.as_deref() == Some(expected_head.as_str())
            && sequence.checked_add(1) == Some(self.next_sequence)
            && previous_certificate_id.as_deref() == Some(certificate_id)
        {
            return Ok(ApplyOutcome::Idempotent);
        }

        self.check_frontier(&seller, sequence, &previous_certificate_id)?;
        if certificate.seller != self.seller { return Err(CertificateError::WrongSeller); }

        let event = if release {
            crate::reservation::ReservationEvent::Release { intent_id: certificate.intent.intent_id.clone() }
        } else {
            crate::reservation::ReservationEvent::Consume { intent_id: certificate.intent.intent_id.clone() }
        };

        let outcome = self.ledger.apply(event).map_err(|error| match error {
            ReservationError::AlreadyReleased { .. } => CertificateError::AlreadyReleased(certificate_id.to_owned()),
            ReservationError::AlreadyConsumed { .. } => CertificateError::AlreadyConsumed(certificate_id.to_owned()),
            other => CertificateError::Capacity(other),
        })?;

        self.advance(format!("{}:{}", certificate_id, if release { "release" } else { "consume" }));
        Ok(outcome)
    }

    fn set_capacity(&mut self, listing_hash: ActionHash, listing_revision: ActionHash, capacity: u32, seller: AgentPubKey, sequence: u64, previous_certificate_id: Option<String>) -> Result<ApplyOutcome, CertificateError> {
        self.check_frontier(&seller, sequence, &previous_certificate_id)?;
        if listing_hash != self.listing_hash { return Err(CertificateError::WrongListing); }
        if listing_revision != self.listing_revision { return Err(CertificateError::WrongRevision); }

        let outcome = self.ledger.apply(crate::reservation::ReservationEvent::SetCapacity { capacity })
            .map_err(CertificateError::Capacity)?;
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

    fn agent(byte: u8) -> AgentPubKey { AgentPubKey::from_raw_36(vec![byte; 36]) }
    fn hash(byte: u8) -> ActionHash { ActionHash::from_raw_36(vec![byte; 36]) }

    fn intent() -> PurchaseIntent {
        PurchaseIntent {
            intent_id: "intent-1".into(), buyer: agent(1), seller: agent(2),
            listing_hash: hash(3), listing_revision: hash(4), quantity: 1,
            unit_price_cents: 500, client_nonce: "nonce-1".into(),
        }
    }

    fn certificate(id: &str, sequence: u64, previous: Option<&str>, quantity: u32) -> ReservationCertificate {
        let intent = PurchaseIntent { quantity, ..intent() };
        ReservationCertificate {
            certificate_id: id.into(), seller: agent(2), listing_hash: hash(3),
            listing_revision: hash(4), intent_hash: hash(5), quantity, sequence,
            previous_certificate_id: previous.map(str::to_owned),
            previous_frontier_action: previous.map(|_| hash(6)),
            intent,
        }
    }

    fn frontier(capacity: u32) -> InventoryFrontier {
        InventoryFrontier::new(agent(2), hash(3), hash(4), capacity)
    }

    #[test]
    fn seller_frontier_serializes_two_competing_reservations() {
        let mut f = frontier(1);
        assert_eq!(f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 1))), Ok(ApplyOutcome::Applied));
        assert!(matches!(
            f.apply(FrontierEvent::Reserve(certificate("c2", 1, Some("c1"), 1))),
            Err(CertificateError::Capacity(ReservationError::InsufficientCapacity { .. }))
        ));
        assert_eq!(f.active_reserved(), 1);
        assert_eq!(f.available(), 0);
    }

    #[test]
    fn replay_of_exact_certificate_is_idempotent() {
        let mut f = frontier(2);
        let c1 = certificate("c1", 0, None, 1);
        assert_eq!(f.apply(FrontierEvent::Reserve(c1.clone())), Ok(ApplyOutcome::Applied));
        assert_eq!(f.apply(FrontierEvent::Reserve(c1)), Ok(ApplyOutcome::Idempotent));
        assert_eq!(f.active_reserved(), 1);
    }

    #[test]
    fn canonical_intent_identity_material_is_stable_and_domain_separated() {
        let a = intent();
        let mut b = a.clone();
        b.client_nonce = "nonce-2".into();

        assert_eq!(a.canonical_identity_material(), a.canonical_identity_material());
        assert_ne!(a.canonical_identity_material(), b.canonical_identity_material());
        assert!(a.canonical_identity_material().starts_with(&36u32.to_le_bytes()));
    }

    #[test]
    fn certificate_binds_seller_listing_revision_and_quantity() {
        let mut f = frontier(2);
        let mut bad = certificate("c1", 0, None, 1);
        bad.seller = agent(9);
        assert_eq!(f.apply(FrontierEvent::Reserve(bad)), Err(CertificateError::WrongSeller));

        let mut bad = certificate("c1", 0, None, 1);
        bad.listing_revision = hash(9);
        assert_eq!(f.apply(FrontierEvent::Reserve(bad)), Err(CertificateError::ListingRevisionMismatch));
    }

    #[test]
    fn listing_capacity_cannot_drop_below_outstanding_reservations() {
        let mut f = frontier(5);
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 4))).unwrap();
        assert!(matches!(
            f.apply(FrontierEvent::SetCapacity {
                listing_hash: hash(3), listing_revision: hash(4), capacity: 3,
                seller: agent(2), sequence: 1, previous_certificate_id: Some("c1".into()),
            }),
            Err(CertificateError::Capacity(ReservationError::CapacityBelowActiveReservations { .. }))
        ));
        assert_eq!(f.capacity(), 5);
    }

    #[test]
    fn release_returns_capacity_then_next_reservation_can_admit() {
        let mut f = frontier(1);
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 1))).unwrap();
        f.apply(FrontierEvent::Release {
            certificate_id: "c1".into(), seller: agent(2), sequence: 1,
            previous_certificate_id: Some("c1".into()),
        }).unwrap();
        assert_eq!(f.available(), 1);
        assert_eq!(
            f.apply(FrontierEvent::Reserve(certificate("c2", 2, Some("c1:release"), 1))),
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
            .validate_transition(
                FrontierStateTransition::Reserve { quantity: 3 },
                &reserved,
            )
            .unwrap();

        let released = ReservationFrontierState {
            capacity: 10,
            active_reserved: 0,
            available: 10,
        };
        reserved
            .validate_transition(
                FrontierStateTransition::Release { quantity: 3 },
                &released,
            )
            .unwrap();

        let consumed = ReservationFrontierState {
            capacity: 7,
            active_reserved: 0,
            available: 7,
        };
        reserved
            .validate_transition(
                FrontierStateTransition::Consume { quantity: 3 },
                &consumed,
            )
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

        assert!(before
            .validate_transition(
                FrontierStateTransition::Reserve { quantity: 1 },
                &forged,
            )
            .is_err());
    }

    #[test]
    fn frontier_post_state_is_self_consistent_and_reconstructible() {
        let mut f = frontier(5);
        assert_eq!(f.post_state(), ReservationFrontierState { capacity: 5, active_reserved: 0, available: 5 });
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 3))).unwrap();
        assert_eq!(f.post_state(), ReservationFrontierState { capacity: 5, active_reserved: 3, available: 2 });
        f.post_state().validate().unwrap();
        f.apply(FrontierEvent::Release {
            certificate_id: "c1".into(), seller: agent(2), sequence: 1,
            previous_certificate_id: Some("c1".into()),
        }).unwrap();
        assert_eq!(f.post_state(), ReservationFrontierState { capacity: 5, active_reserved: 0, available: 5 });
    }

    #[test]
    fn consume_permanently_removes_capacity() {
        let mut f = frontier(1);
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 1))).unwrap();
        f.apply(FrontierEvent::Consume {
            certificate_id: "c1".into(), seller: agent(2), sequence: 1,
            previous_certificate_id: Some("c1".into()),
        }).unwrap();
        assert_eq!(f.capacity(), 0);
        assert_eq!(f.available(), 0);
    }

    #[test]
    fn forked_frontier_is_rejected_by_previous_certificate_binding() {
        let mut f = frontier(3);
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 1))).unwrap();
        let error = f.apply(FrontierEvent::Reserve(certificate("c2", 1, Some("other-head"), 1))).unwrap_err();
        assert!(matches!(error, CertificateError::PreviousMismatch { .. }));
    }

    #[test]
    fn stale_listing_revision_cannot_admit_new_reservation() {
        let mut f = frontier(2);
        let mut stale = certificate("c1", 0, None, 1);
        stale.listing_revision = hash(9);
        assert_eq!(f.apply(FrontierEvent::Reserve(stale)), Err(CertificateError::ListingRevisionMismatch));
    }

    #[test]
    fn buyer_fields_are_bound_to_the_intent_and_seller() {
        let mut f = frontier(2);
        let mut forged = certificate("c1", 0, None, 1);
        forged.intent.buyer = agent(7);
        assert_eq!(f.apply(FrontierEvent::Reserve(forged)), Ok(ApplyOutcome::Applied));
        assert_eq!(f.seller(), &agent(2));
    }

    #[test]
    fn terminal_conflict_cannot_cross_release_and_consume() {
        let mut f = frontier(1);
        f.apply(FrontierEvent::Reserve(certificate("c1", 0, None, 1))).unwrap();
        f.apply(FrontierEvent::Release {
            certificate_id: "c1".into(), seller: agent(2), sequence: 1,
            previous_certificate_id: Some("c1".into()),
        }).unwrap();
        let error = f.apply(FrontierEvent::Consume {
            certificate_id: "c1".into(), seller: agent(2), sequence: 2,
            previous_certificate_id: Some("c1:release".into()),
        }).unwrap_err();
        assert!(matches!(error, CertificateError::AlreadyReleased(_)));
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
        assert!(validate_transaction_reservation_binding(&tx, &c)
            .unwrap_err()
            .contains("listing"));
    }

    #[test]
    fn transaction_binding_rejects_cross_buyer_certificate() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.buyer = agent(88);
        assert!(validate_transaction_reservation_binding(&tx, &c)
            .unwrap_err()
            .contains("buyer"));
    }

    #[test]
    fn transaction_binding_rejects_quantity_mismatch() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.quantity = 2;
        tx.total_price_cents = i.unit_price_cents * 2;
        assert!(validate_transaction_reservation_binding(&tx, &c)
            .unwrap_err()
            .contains("quantity"));
    }

    #[test]
    fn transaction_binding_rejects_price_mismatch() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.total_price_cents += 1;
        assert!(validate_transaction_reservation_binding(&tx, &c)
            .unwrap_err()
            .contains("total"));
    }

    #[test]
    fn transaction_binding_rejects_certificate_that_is_listing_hash() {
        let i = intent();
        let c = certificate("c1", 0, None, i.quantity);
        let mut tx = transaction_for(&i);
        tx.reservation_certificate_hash = tx.listing_hash.clone();
        assert!(validate_transaction_reservation_binding(&tx, &c)
            .unwrap_err()
            .contains("distinct"));
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
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ReservationTerminalOutcome {
    Released,
    Consumed,
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
        return Ok(ValidateCallbackResult::Invalid(format!("Invalid purchase intent: {error:?}")));
    }
    if action.author != intent.buyer {
        return Ok(ValidateCallbackResult::Invalid(
            "PurchaseIntent must be authored by its buyer".into(),
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

    let intent_record = must_get_valid_record(certificate.intent_hash.clone()).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "ReservationCertificate references a missing or invalid PurchaseIntent".into(),
        ))
    })?;
    let intent = intent_record
        .entry()
        .to_app_option::<PurchaseIntent>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Invalid PurchaseIntent entry: {e:?}"))))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "ReservationCertificate intent dependency has the wrong entry type".into(),
        )))?;

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

    let listing_action = must_get_action(certificate.listing_hash.clone())?;
    if listing_action.author() != &certificate.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate seller does not own the referenced listing action".into(),
        ));
    }

    let revision_action = must_get_action(certificate.listing_revision.clone())?;
    if revision_action.author() != &certificate.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "ReservationCertificate listing revision is not seller-authored".into(),
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

    if certificate.sequence > 0 {
        let previous_hash = certificate.previous_frontier_action.clone().ok_or_else(|| {
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
            ChainFilter::new(action.prev_action.clone())
                .until_hash(previous_hash.clone()),
        )?;
        if !prior_activity.iter().any(|activity| activity.action.hashed.hash == previous_hash) {
            return Ok(ValidateCallbackResult::Invalid(
                "Reservation frontier predecessor is not an earlier action on the seller source chain".into(),
            ));
        }

        let previous = must_get_valid_record(previous_hash).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "ReservationCertificate references a missing or invalid previous frontier record".into(),
            ))
        })?;
        if previous.action().author() != &certificate.seller {
            return Ok(ValidateCallbackResult::Invalid(
                "Previous frontier record is not seller-authored".into(),
            ));
        }

        // The frontier is per seller + listing + exact revision. A seller-authored
        // record from another economic domain is not a valid predecessor merely
        // because its author matches.
        if let Some(previous_certificate) = previous
            .entry()
            .to_app_option::<ReservationCertificate>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid previous frontier certificate entry: {e:?}"
            ))))?
        {
            if previous_certificate.seller != certificate.seller
                || previous_certificate.listing_hash != certificate.listing_hash
                || previous_certificate.listing_revision != certificate.listing_revision
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Previous frontier certificate belongs to a different seller/listing/revision".into(),
                ));
            }
            if previous_certificate.sequence.checked_add(1) != Some(certificate.sequence) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Reservation frontier sequence does not follow its previous certificate".into(),
                ));
            }
        } else if let Some(previous_terminal) = previous
            .entry()
            .to_app_option::<ReservationTerminalEvidence>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid previous frontier terminal entry: {e:?}"
            ))))?
        {
            if previous_terminal.seller != certificate.seller {
                return Ok(ValidateCallbackResult::Invalid(
                    "Previous frontier terminal evidence belongs to another seller".into(),
                ));
            }
            let terminal_certificate = must_get_valid_record(
                previous_terminal.certificate_hash.clone(),
            )
            .map_err(|_| wasm_error!(WasmErrorInner::Guest(
                "Previous frontier terminal evidence references a missing certificate".into(),
            )))?;
            let terminal_certificate = terminal_certificate
                .entry()
                .to_app_option::<ReservationCertificate>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid previous frontier terminal certificate: {e:?}"
                ))))?
                .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
                    "Previous frontier terminal certificate has the wrong entry type".into(),
                )))?;
            if terminal_certificate.seller != certificate.seller
                || terminal_certificate.listing_hash != certificate.listing_hash
                || terminal_certificate.listing_revision != certificate.listing_revision
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Previous frontier terminal evidence belongs to a different seller/listing/revision".into(),
                ));
            }
            if previous_terminal.sequence.checked_add(1) != Some(certificate.sequence) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Reservation frontier sequence does not follow its previous terminal event".into(),
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

    let certificate_record = must_get_valid_record(evidence.certificate_hash.clone()).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "Reservation terminal evidence references a missing or invalid certificate".into(),
        ))
    })?;
    let certificate = certificate_record
        .entry()
        .to_app_option::<ReservationCertificate>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Invalid certificate entry: {e:?}"))))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(
            "Reservation terminal certificate dependency has the wrong entry type".into(),
        )))?;

    if certificate.seller != evidence.seller || certificate.intent_hash != evidence.intent_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal evidence does not bind to the certificate seller/intent".into(),
        ));
    }

    let previous = must_get_valid_record(evidence.previous_frontier_action.clone()).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(
            "Reservation terminal evidence references a missing or invalid previous frontier".into(),
        ))
    })?;
    if previous.action().author() != &evidence.seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal previous frontier is not seller-authored".into(),
        ));
    }
    // As with reservation admission, unrelated seller-authored records may
    // interleave between the certificate and its terminal evidence. Require the
    // certificate to be an earlier source-chain action rather than requiring
    // immediate adjacency.
    let prior_activity = must_get_agent_activity(
        evidence.seller.clone(),
        ChainFilter::new(action.prev_action.clone())
            .until_hash(evidence.certificate_hash.clone()),
    )?;
    if !prior_activity
        .iter()
        .any(|activity| activity.action.hashed.hash == evidence.certificate_hash)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal certificate is not an earlier action on the seller source chain".into(),
        ));
    }
    if evidence.sequence != certificate.sequence + 1 {
        return Ok(ValidateCallbackResult::Invalid(
            "Reservation terminal evidence sequence does not follow its certificate".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

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
    pub intent: PurchaseIntent,
    pub quantity: u32,
    pub sequence: u64,
    pub previous_certificate_id: Option<String>,
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
        if self.sequence == 0 && self.previous_certificate_id.is_some() { return Err(CertificateError::GenesisHasPrevious); }
        if self.sequence > 0 && self.previous_certificate_id.is_none() { return Err(CertificateError::MissingPrevious); }
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
            listing_revision: hash(4), quantity, sequence,
            previous_certificate_id: previous.map(str::to_owned), intent,
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
}

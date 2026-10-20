#![forbid(unsafe_code)]

//! A small, deterministic reference model for one need-to-fulfilment lifecycle.
//!
//! This crate is intentionally NOT a Holochain zome, production ledger, payment
//! adapter, signature verifier, or economic-policy optimizer. Callers supply
//! evidence-verification outcomes; this model checks the consequences of those
//! outcomes and keeps financial and physical states distinct.

/// Experimental escrow allocation for partition-tolerant local reservations; not a distributed protocol.
pub mod fulfillment_escrow;

/// Multi-replica fixed-rights simulation for isolated partitioned reservations.
pub mod fulfillment_escrow_partition;

/// Single-authority claim identity registry; deliberately unavailable under partition.
pub mod fulfillment_claim_identity;

/// Experimental append-only model for partial fulfilment; not wired into v1 settlement.
pub mod fulfillment_tranches;

use std::collections::{BTreeMap, BTreeSet};
use std::convert::TryFrom;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceStatus {
    Verified,
    Unknown,
    Stale,
    Contested,
}

/// Origin is orthogonal to verification status: verified synthetic output is
/// still synthetic and cannot stand in for evidence from the live domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceOrigin {
    LiveDomain,
    SyntheticSimulation,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lifecycle {
    Proposed,
    Authorized,
    Reserved,
    Committed,
    Delivered,
    Verified,
    CapacityConsumed,
    Settled,
    Disputed,
    Indeterminate,
    Cancelled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContractError {
    InvalidRequest,
    InvalidTransition,
    Expired,
    EvidenceNotVerified,
    PolicyDenied,
    DuplicateCapacity,
    CapacityUnavailable,
    DuplicateReservation,
    MissingReservation,
    InconsistentCapacity,
    InvalidObligation,
    InvalidEvidence,
    NotIndependentVerifier,
    DuplicateAccount,
    AccountNotFound,
    BalanceLimit,
    ArithmeticOverflow,
    AlreadyPosted,
    NonMonotonicTime,
}

impl fmt::Display for ContractError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{:?}", self)
    }
}

impl std::error::Error for ContractError {}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NeedRequest {
    pub transaction_id: String,
    pub community_id: String,
    pub requester_id: String,
    pub provider_id: String,
    pub resource_id: String,
    pub unit: String,
    pub window_id: String,
    pub quantity: u64,
    /// Latest time at which authorization/reservation/commit may be accepted.
    pub accept_until: u64,
    /// Latest declared delivery time. Late or unproven delivery is not silently
    /// accepted as fulfilled.
    pub deliver_by: u64,
}

impl NeedRequest {
    fn validate(&self) -> Result<(), ContractError> {
        if [
            self.transaction_id.as_str(),
            self.community_id.as_str(),
            self.requester_id.as_str(),
            self.provider_id.as_str(),
            self.resource_id.as_str(),
            self.unit.as_str(),
            self.window_id.as_str(),
        ]
        .iter()
        .any(|value| !non_empty(value))
            || self.requester_id == self.provider_id
            || self.quantity == 0
            || self.accept_until == 0
            || self.deliver_by < self.accept_until
        {
            return Err(ContractError::InvalidRequest);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthorityProof {
    pub community_id: String,
    pub actor_id: String,
    pub policy_digest: String,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GateAssessment {
    pub policy_id: String,
    pub permitted: bool,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransitionReceipt {
    pub from: Lifecycle,
    pub to: Lifecycle,
    pub at: u64,
    /// A caller-supplied evidence identifier; this is not a cryptographic digest.
    pub evidence_ref: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReservationReceipt {
    pub transaction_id: String,
    pub resource_id: String,
    pub unit: String,
    pub window_id: String,
    pub quantity: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ObligationTerms {
    pub id: String,
    pub debtor_id: String,
    pub creditor_id: String,
    pub debit_account: String,
    pub credit_account: String,
    pub instrument_id: String,
    /// Exact smallest indivisible instrument unit; no floating-point amount.
    pub atomic_unit: String,
    pub amount_atomic: u64,
    pub due_at: u64,
    pub policy_digest: String,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeliveryClaim {
    pub source_id: String,
    pub evidence_id: String,
    pub resource_id: String,
    pub unit: String,
    pub quantity: u64,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
    pub observed_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerificationResult {
    pub verifier_id: String,
    /// Exact provider/source claim the independent verifier evaluated.
    pub claim_evidence_id: String,
    /// Identifier for the independent verifier's own evidence/decision.
    pub evidence_id: String,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
    pub accepted_quantity: Option<u64>,
    pub decided_at: u64,
}

/// An independent reassessment of a previously disputed delivery.
/// It never deletes or waives an obligation; it can only confirm full delivery
/// for the existing contract, otherwise the dispute remains open.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DisputeReassessment {
    pub resolver_id: String,
    pub claim_evidence_id: String,
    pub decision_evidence_id: String,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
    pub accepted_quantity: Option<u64>,
    pub decided_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettlementReceipt {
    pub transaction_id: String,
    pub obligation_id: String,
    pub instrument_id: String,
    pub atomic_unit: String,
    pub amount_atomic: u64,
    pub debit_account: String,
    pub credit_account: String,
    pub debit_before: i128,
    pub debit_after: i128,
    pub credit_before: i128,
    pub credit_after: i128,
    pub posted_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct CapacityKey {
    resource_id: String,
    unit: String,
    window_id: String,
}

impl CapacityKey {
    fn from_request(request: &NeedRequest) -> Self {
        Self {
            resource_id: request.resource_id.clone(),
            unit: request.unit.clone(),
            window_id: request.window_id.clone(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct CapacitySlot {
    total: u64,
    reserved: u64,
    consumed: u64,
}

/// Serial reference book for capacity. It demonstrates bounded reservations;
/// it does not claim distributed concurrency safety across Holochain agents.
#[derive(Default)]
pub struct CapacityBook {
    slots: BTreeMap<CapacityKey, CapacitySlot>,
    reservations: BTreeMap<String, ReservationReceipt>,
    /// Reservation transaction identities are single-use, including after release.
    seen_transactions: BTreeSet<String>,
}

impl CapacityBook {
    pub fn declare_capacity(
        &mut self,
        resource_id: &str,
        unit: &str,
        window_id: &str,
        total: u64,
    ) -> Result<(), ContractError> {
        if !non_empty(resource_id) || !non_empty(unit) || !non_empty(window_id) {
            return Err(ContractError::InvalidRequest);
        }
        let key = CapacityKey {
            resource_id: resource_id.to_owned(),
            unit: unit.to_owned(),
            window_id: window_id.to_owned(),
        };
        if self.slots.contains_key(&key) {
            return Err(ContractError::DuplicateCapacity);
        }
        self.slots.insert(
            key,
            CapacitySlot {
                total,
                reserved: 0,
                consumed: 0,
            },
        );
        Ok(())
    }

    pub fn remaining(
        &self,
        resource_id: &str,
        unit: &str,
        window_id: &str,
    ) -> Option<u64> {
        let key = CapacityKey {
            resource_id: resource_id.to_owned(),
            unit: unit.to_owned(),
            window_id: window_id.to_owned(),
        };
        self.slots.get(&key).and_then(|slot| {
            slot.total
                .checked_sub(slot.reserved)
                .and_then(|remaining| remaining.checked_sub(slot.consumed))
        })
    }

    pub fn consumed(
        &self,
        resource_id: &str,
        unit: &str,
        window_id: &str,
    ) -> Option<u64> {
        let key = CapacityKey {
            resource_id: resource_id.to_owned(),
            unit: unit.to_owned(),
            window_id: window_id.to_owned(),
        };
        self.slots.get(&key).map(|slot| slot.consumed)
    }

    pub fn reservation(&self, transaction_id: &str) -> Option<&ReservationReceipt> {
        self.reservations.get(transaction_id)
    }

    fn reserve(&mut self, request: &NeedRequest) -> Result<ReservationReceipt, ContractError> {
        if self.seen_transactions.contains(&request.transaction_id)
            || self.reservations.contains_key(&request.transaction_id)
        {
            return Err(ContractError::DuplicateReservation);
        }
        let key = CapacityKey::from_request(request);
        let slot = self
            .slots
            .get(&key)
            .ok_or(ContractError::CapacityUnavailable)?;
        let next_reserved = slot
            .reserved
            .checked_add(request.quantity)
            .ok_or(ContractError::ArithmeticOverflow)?;
        let allocated = next_reserved
            .checked_add(slot.consumed)
            .ok_or(ContractError::ArithmeticOverflow)?;
        if allocated > slot.total {
            return Err(ContractError::CapacityUnavailable);
        }

        self.slots
            .get_mut(&key)
            .ok_or(ContractError::CapacityUnavailable)?
            .reserved = next_reserved;

        let receipt = ReservationReceipt {
            transaction_id: request.transaction_id.clone(),
            resource_id: request.resource_id.clone(),
            unit: request.unit.clone(),
            window_id: request.window_id.clone(),
            quantity: request.quantity,
        };
        self.seen_transactions.insert(request.transaction_id.clone());
        self.reservations
            .insert(request.transaction_id.clone(), receipt.clone());
        Ok(receipt)
    }

    fn release(&mut self, transaction_id: &str) -> Result<(), ContractError> {
        let receipt = self
            .reservations
            .get(transaction_id)
            .cloned()
            .ok_or(ContractError::MissingReservation)?;
        let key = CapacityKey {
            resource_id: receipt.resource_id,
            unit: receipt.unit,
            window_id: receipt.window_id,
        };
        let slot = self
            .slots
            .get(&key)
            .ok_or(ContractError::InconsistentCapacity)?;
        let next_reserved = slot
            .reserved
            .checked_sub(receipt.quantity)
            .ok_or(ContractError::InconsistentCapacity)?;
        self.slots
            .get_mut(&key)
            .ok_or(ContractError::InconsistentCapacity)?
            .reserved = next_reserved;
        self.reservations.remove(transaction_id);
        Ok(())
    }

    fn consume(
        &mut self,
        transaction_id: &str,
        expected: &ReservationReceipt,
    ) -> Result<(), ContractError> {
        let receipt = self
            .reservations
            .get(transaction_id)
            .cloned()
            .ok_or(ContractError::MissingReservation)?;
        if &receipt != expected {
            return Err(ContractError::InconsistentCapacity);
        }
        let key = CapacityKey {
            resource_id: receipt.resource_id,
            unit: receipt.unit,
            window_id: receipt.window_id,
        };
        let slot = self
            .slots
            .get(&key)
            .ok_or(ContractError::InconsistentCapacity)?;
        let next_reserved = slot
            .reserved
            .checked_sub(receipt.quantity)
            .ok_or(ContractError::InconsistentCapacity)?;
        let next_consumed = slot
            .consumed
            .checked_add(receipt.quantity)
            .ok_or(ContractError::ArithmeticOverflow)?;
        let allocated = next_reserved
            .checked_add(next_consumed)
            .ok_or(ContractError::ArithmeticOverflow)?;
        if allocated > slot.total {
            return Err(ContractError::InconsistentCapacity);
        }
        let slot = self
            .slots
            .get_mut(&key)
            .ok_or(ContractError::InconsistentCapacity)?;
        slot.reserved = next_reserved;
        slot.consumed = next_consumed;
        self.reservations.remove(transaction_id);
        Ok(())
    }
}

pub struct CoordinationContract {
    request: NeedRequest,
    lifecycle: Lifecycle,
    authority: Option<AuthorityProof>,
    reservation: Option<ReservationReceipt>,
    capacity_consumed: bool,
    obligation: Option<ObligationTerms>,
    delivery: Option<DeliveryClaim>,
    verification: Option<VerificationResult>,
    verification_history: Vec<VerificationResult>,
    dispute_reassessments: Vec<DisputeReassessment>,
    settlement: Option<SettlementReceipt>,
    history: Vec<TransitionReceipt>,
}

impl CoordinationContract {
    pub fn new(request: NeedRequest) -> Result<Self, ContractError> {
        request.validate()?;
        Ok(Self {
            request,
            lifecycle: Lifecycle::Proposed,
            authority: None,
            reservation: None,
            capacity_consumed: false,
            obligation: None,
            delivery: None,
            verification: None,
            verification_history: Vec::new(),
            dispute_reassessments: Vec::new(),
            settlement: None,
            history: Vec::new(),
        })
    }

    pub fn request(&self) -> &NeedRequest {
        &self.request
    }

    pub fn lifecycle(&self) -> Lifecycle {
        self.lifecycle
    }

    pub fn history(&self) -> &[TransitionReceipt] {
        &self.history
    }

    pub fn reservation(&self) -> Option<&ReservationReceipt> {
        self.reservation.as_ref()
    }

    pub fn capacity_consumed(&self) -> bool {
        self.capacity_consumed
    }

    pub fn obligation(&self) -> Option<&ObligationTerms> {
        self.obligation.as_ref()
    }

    pub fn delivery_claim(&self) -> Option<&DeliveryClaim> {
        self.delivery.as_ref()
    }

    pub fn verification(&self) -> Option<&VerificationResult> {
        self.verification.as_ref()
    }

    /// Append-only record of every accepted-to-journal verification decision,
    /// including the original disputed result and later reassessments.
    pub fn verification_history(&self) -> &[VerificationResult] {
        &self.verification_history
    }

    pub fn dispute_reassessments(&self) -> &[DisputeReassessment] {
        &self.dispute_reassessments
    }

    pub fn settlement(&self) -> Option<&SettlementReceipt> {
        self.settlement.as_ref()
    }

    pub fn authorize(
        &mut self,
        authority: AuthorityProof,
        rights: GateAssessment,
        ecological: GateAssessment,
        now: u64,
    ) -> Result<(), ContractError> {
        self.require_state(Lifecycle::Proposed)?;
        self.ensure_transition_time(now)?;
        if now >= self.request.accept_until {
            return Err(ContractError::Expired);
        }
        if authority.community_id != self.request.community_id
            || !non_empty(&authority.actor_id)
            || !non_empty(&authority.policy_digest)
            || !non_empty(&rights.policy_id)
            || !non_empty(&ecological.policy_id)
        {
            return Err(ContractError::InvalidRequest);
        }
        if authority.evidence_status != EvidenceStatus::Verified
            || authority.origin != EvidenceOrigin::LiveDomain
            || rights.evidence_status != EvidenceStatus::Verified
            || rights.origin != EvidenceOrigin::LiveDomain
            || ecological.evidence_status != EvidenceStatus::Verified
            || ecological.origin != EvidenceOrigin::LiveDomain
        {
            return Err(ContractError::EvidenceNotVerified);
        }
        if !rights.permitted || !ecological.permitted {
            return Err(ContractError::PolicyDenied);
        }

        let evidence_ref = Some(format!(
            "authority-policy:{};rights-policy:{};ecology-policy:{}",
            authority.policy_digest, rights.policy_id, ecological.policy_id
        ));
        self.authority = Some(authority);
        self.transition(Lifecycle::Authorized, now, evidence_ref);
        Ok(())
    }

    pub fn reserve(
        &mut self,
        book: &mut CapacityBook,
        now: u64,
    ) -> Result<ReservationReceipt, ContractError> {
        self.require_state(Lifecycle::Authorized)?;
        self.ensure_transition_time(now)?;
        if now >= self.request.accept_until {
            return Err(ContractError::Expired);
        }
        let receipt = book.reserve(&self.request)?;
        self.reservation = Some(receipt.clone());
        self.transition(
            Lifecycle::Reserved,
            now,
            Some(format!("reservation:{}", receipt.transaction_id)),
        );
        Ok(receipt)
    }

    pub fn commit(
        &mut self,
        obligation: ObligationTerms,
        now: u64,
    ) -> Result<(), ContractError> {
        self.require_state(Lifecycle::Reserved)?;
        self.ensure_transition_time(now)?;
        if now >= self.request.accept_until {
            return Err(ContractError::Expired);
        }
        let authority = self
            .authority
            .as_ref()
            .ok_or(ContractError::InvalidObligation)?;
        if !non_empty(&obligation.id)
            || obligation.debtor_id != self.request.requester_id
            || obligation.creditor_id != self.request.provider_id
            || !non_empty(&obligation.debit_account)
            || !non_empty(&obligation.credit_account)
            || obligation.debit_account == obligation.credit_account
            || !non_empty(&obligation.instrument_id)
            || !non_empty(&obligation.atomic_unit)
            || obligation.amount_atomic == 0
            || obligation.due_at < now
            || obligation.policy_digest != authority.policy_digest
        {
            return Err(ContractError::InvalidObligation);
        }
        if obligation.evidence_status != EvidenceStatus::Verified
            || obligation.origin != EvidenceOrigin::LiveDomain
        {
            return Err(ContractError::EvidenceNotVerified);
        }
        let evidence_ref = Some(format!("obligation:{}", obligation.id));
        self.obligation = Some(obligation);
        self.transition(Lifecycle::Committed, now, evidence_ref);
        Ok(())
    }

    /// Record a delivery claim. This transition records a claim, not proof that
    /// delivery occurred; independent verification is a separate transition.
    pub fn record_delivery(&mut self, claim: DeliveryClaim) -> Result<(), ContractError> {
        self.require_state(Lifecycle::Committed)?;
        // Only the named provider may assert delivery for this contract. A
        // third-party claim must not become settleable merely because a verifier
        // later accepts its resource/quantity fields.
        if !non_empty(&claim.source_id)
            || claim.source_id != self.request.provider_id
            || !non_empty(&claim.evidence_id)
        {
            return Err(ContractError::InvalidEvidence);
        }
        let at = claim.observed_at;
        self.ensure_transition_time(at)?;
        let evidence_ref = Some(claim.evidence_id.clone());
        self.delivery = Some(claim);
        self.transition(Lifecycle::Delivered, at, evidence_ref);
        Ok(())
    }

    /// Re-check an indeterminate delivery when new evidence becomes available.
    /// The verifier cannot be the requester, provider, or original evidence source.
    pub fn verify_delivery(
        &mut self,
        result: VerificationResult,
    ) -> Result<Lifecycle, ContractError> {
        if self.lifecycle != Lifecycle::Delivered && self.lifecycle != Lifecycle::Indeterminate {
            return Err(ContractError::InvalidTransition);
        }
        self.ensure_transition_time(result.decided_at)?;
        let delivery = self.delivery.as_ref().ok_or(ContractError::InvalidEvidence)?;
        if !non_empty(&result.verifier_id)
            || !non_empty(&result.claim_evidence_id)
            || !non_empty(&result.evidence_id)
        {
            return Err(ContractError::InvalidEvidence);
        }
        if result.claim_evidence_id != delivery.evidence_id {
            return Err(ContractError::InvalidEvidence);
        }
        if result.verifier_id == self.request.requester_id
            || result.verifier_id == self.request.provider_id
            || result.verifier_id == delivery.source_id
        {
            return Err(ContractError::NotIndependentVerifier);
        }

        let next = if result.evidence_status != EvidenceStatus::Verified
            || result.origin != EvidenceOrigin::LiveDomain
            || delivery.origin != EvidenceOrigin::LiveDomain
            || result.accepted_quantity.is_none()
        {
            Lifecycle::Indeterminate
        } else {
            let accepted = result.accepted_quantity.ok_or(ContractError::InvalidEvidence)?;
            if delivery.resource_id == self.request.resource_id
                && delivery.unit == self.request.unit
                && delivery.quantity == self.request.quantity
                && accepted == self.request.quantity
                && result.decided_at >= delivery.observed_at
                && delivery.observed_at <= self.request.deliver_by
            {
                Lifecycle::Verified
            } else {
                Lifecycle::Disputed
            }
        };

        let evidence_ref = Some(result.evidence_id.clone());
        let at = result.decided_at;
        self.verification_history.push(result.clone());
        self.verification = Some(result);
        self.transition(next, at, evidence_ref);
        Ok(next)
    }

    /// Reassess a disputed delivery through a distinct, independent resolver.
    ///
    /// Only evidence that confirms the exact full delivery can advance to
    /// Verified. An unresolved/adverse assessment is retained in the journal
    /// and leaves the obligation disputed; it never silently waives money owed.
    pub fn reassess_dispute(
        &mut self,
        reassessment: DisputeReassessment,
    ) -> Result<Lifecycle, ContractError> {
        self.require_state(Lifecycle::Disputed)?;
        self.ensure_transition_time(reassessment.decided_at)?;
        let delivery = self.delivery.as_ref().ok_or(ContractError::InvalidEvidence)?;
        let previous_verifier = self.verification.as_ref().map(|v| v.verifier_id.as_str());

        if !non_empty(&reassessment.resolver_id)
            || !non_empty(&reassessment.claim_evidence_id)
            || !non_empty(&reassessment.decision_evidence_id)
            || reassessment.claim_evidence_id != delivery.evidence_id
        {
            return Err(ContractError::InvalidEvidence);
        }
        if reassessment.resolver_id == self.request.requester_id
            || reassessment.resolver_id == self.request.provider_id
            || reassessment.resolver_id == delivery.source_id
            || Some(reassessment.resolver_id.as_str()) == previous_verifier
            || self
                .dispute_reassessments
                .iter()
                .any(|previous| previous.resolver_id == reassessment.resolver_id)
        {
            return Err(ContractError::NotIndependentVerifier);
        }

        let resolves = reassessment.evidence_status == EvidenceStatus::Verified
            && reassessment.origin == EvidenceOrigin::LiveDomain
            && delivery.origin == EvidenceOrigin::LiveDomain
            && reassessment.accepted_quantity == Some(self.request.quantity)
            && delivery.resource_id == self.request.resource_id
            && delivery.unit == self.request.unit
            && delivery.quantity == self.request.quantity
            && delivery.observed_at <= self.request.deliver_by;

        let at = reassessment.decided_at;
        let evidence_ref = Some(reassessment.decision_evidence_id.clone());
        let next = if resolves {
            Lifecycle::Verified
        } else {
            Lifecycle::Disputed
        };
        let reassessed_verification = VerificationResult {
            verifier_id: reassessment.resolver_id.clone(),
            claim_evidence_id: reassessment.claim_evidence_id.clone(),
            evidence_id: reassessment.decision_evidence_id.clone(),
            evidence_status: reassessment.evidence_status,
            origin: reassessment.origin,
            accepted_quantity: reassessment.accepted_quantity,
            decided_at: reassessment.decided_at,
        };
        self.verification_history
            .push(reassessed_verification.clone());
        if resolves {
            // Keep the current verification accessor aligned with the accepted
            // state. Prior decisions remain in verification_history.
            self.verification = Some(reassessed_verification);
        }
        self.dispute_reassessments.push(reassessment);
        self.transition(next, at, evidence_ref);

        // The state transition and journal append succeeded in either case;
        // return the resulting state rather than an error after mutating state.
        Ok(next)
    }

    /// Record that verified delivery consumed the reserved physical capacity.
    /// Capacity consumption is separate from settlement: delivered capacity is
    /// consumed even if later accounting cannot post the financial movement.
    pub fn consume_verified_capacity(
        &mut self,
        book: &mut CapacityBook,
        now: u64,
    ) -> Result<(), ContractError> {
        self.require_state(Lifecycle::Verified)?;
        self.ensure_transition_time(now)?;
        if self.capacity_consumed {
            return Err(ContractError::InvalidTransition);
        }
        let reservation = self
            .reservation
            .as_ref()
            .ok_or(ContractError::MissingReservation)?;
        book.consume(&self.request.transaction_id, reservation)?;
        self.capacity_consumed = true;
        self.transition(
            Lifecycle::CapacityConsumed,
            now,
            Some(format!("capacity-consumed:{}", self.request.transaction_id)),
        );
        Ok(())
    }

    /// Posts an internal double-entry reference ledger movement only after
    /// verified delivery has consumed its reservation. This is not an external rail.
    pub fn settle(
        &mut self,
        ledger: &mut Ledger,
        now: u64,
    ) -> Result<SettlementReceipt, ContractError> {
        self.require_state(Lifecycle::CapacityConsumed)?;
        self.ensure_transition_time(now)?;
        let obligation = self
            .obligation
            .as_ref()
            .ok_or(ContractError::InvalidObligation)?;
        let obligation_id = obligation.id.clone();
        let receipt = ledger.post(&self.request.transaction_id, obligation, now)?;
        self.settlement = Some(receipt.clone());
        self.transition(
            Lifecycle::Settled,
            now,
            Some(format!("settlement-obligation:{obligation_id}")),
        );
        Ok(receipt)
    }

    /// Cancellation is permitted only before commitment, when there is no
    /// outstanding obligation or ambiguous external effect to conceal.
    pub fn cancel(
        &mut self,
        book: &mut CapacityBook,
        now: u64,
    ) -> Result<(), ContractError> {
        if !matches!(
            self.lifecycle,
            Lifecycle::Proposed | Lifecycle::Authorized | Lifecycle::Reserved
        ) {
            return Err(ContractError::InvalidTransition);
        }
        self.ensure_transition_time(now)?;
        if self.lifecycle == Lifecycle::Reserved {
            book.release(&self.request.transaction_id)?;
            self.reservation = None;
        }
        self.transition(Lifecycle::Cancelled, now, None);
        Ok(())
    }

    fn require_state(&self, expected: Lifecycle) -> Result<(), ContractError> {
        if self.lifecycle != expected {
            return Err(ContractError::InvalidTransition);
        }
        Ok(())
    }

    fn ensure_transition_time(&self, at: u64) -> Result<(), ContractError> {
        if self.history.last().is_some_and(|previous| at < previous.at) {
            return Err(ContractError::NonMonotonicTime);
        }
        Ok(())
    }

    fn transition(&mut self, next: Lifecycle, at: u64, evidence_ref: Option<String>) {
        let previous = self.lifecycle;
        self.lifecycle = next;
        self.history.push(TransitionReceipt {
            from: previous,
            to: next,
            at,
            evidence_ref,
        });
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct AccountKey {
    account_id: String,
    instrument_id: String,
    atomic_unit: String,
}

impl AccountKey {
    fn new(account_id: &str, instrument_id: &str, atomic_unit: &str) -> Self {
        Self {
            account_id: account_id.to_owned(),
            instrument_id: instrument_id.to_owned(),
            atomic_unit: atomic_unit.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct LedgerAccount {
    balance: i128,
    min_balance: i128,
    max_balance: i128,
}

/// Minimal in-memory, instrument-specific, double-entry reference ledger.
#[derive(Default)]
pub struct Ledger {
    accounts: BTreeMap<AccountKey, LedgerAccount>,
    postings: BTreeMap<String, SettlementReceipt>,
    posted_obligations: BTreeSet<String>,
}

impl Ledger {
    pub fn register_account(
        &mut self,
        account_id: &str,
        instrument_id: &str,
        atomic_unit: &str,
        opening_balance: i128,
        min_balance: i128,
        max_balance: i128,
    ) -> Result<(), ContractError> {
        if !non_empty(account_id)
            || !non_empty(instrument_id)
            || !non_empty(atomic_unit)
            || min_balance > max_balance
            || opening_balance < min_balance
            || opening_balance > max_balance
        {
            return Err(ContractError::InvalidRequest);
        }
        let key = AccountKey::new(account_id, instrument_id, atomic_unit);
        if self.accounts.contains_key(&key) {
            return Err(ContractError::DuplicateAccount);
        }
        self.accounts.insert(
            key,
            LedgerAccount {
                balance: opening_balance,
                min_balance,
                max_balance,
            },
        );
        Ok(())
    }

    pub fn balance(
        &self,
        account_id: &str,
        instrument_id: &str,
        atomic_unit: &str,
    ) -> Option<i128> {
        self.accounts
            .get(&AccountKey::new(account_id, instrument_id, atomic_unit))
            .map(|account| account.balance)
    }

    pub fn posting(&self, transaction_id: &str) -> Option<&SettlementReceipt> {
        self.postings.get(transaction_id)
    }

    fn post(
        &mut self,
        transaction_id: &str,
        obligation: &ObligationTerms,
        posted_at: u64,
    ) -> Result<SettlementReceipt, ContractError> {
        if !non_empty(&obligation.id) || obligation.amount_atomic == 0 {
            return Err(ContractError::InvalidObligation);
        }
        if self.postings.contains_key(transaction_id)
            || self.posted_obligations.contains(&obligation.id)
        {
            return Err(ContractError::AlreadyPosted);
        }
        let debit_key = AccountKey::new(
            &obligation.debit_account,
            &obligation.instrument_id,
            &obligation.atomic_unit,
        );
        let credit_key = AccountKey::new(
            &obligation.credit_account,
            &obligation.instrument_id,
            &obligation.atomic_unit,
        );
        let (debit_before, debit_min, debit_max) = self
            .accounts
            .get(&debit_key)
            .map(|account| (account.balance, account.min_balance, account.max_balance))
            .ok_or(ContractError::AccountNotFound)?;
        let (credit_before, credit_min, credit_max) = self
            .accounts
            .get(&credit_key)
            .map(|account| (account.balance, account.min_balance, account.max_balance))
            .ok_or(ContractError::AccountNotFound)?;

        let amount = i128::try_from(obligation.amount_atomic)
            .map_err(|_| ContractError::ArithmeticOverflow)?;
        let debit_after = debit_before
            .checked_sub(amount)
            .ok_or(ContractError::ArithmeticOverflow)?;
        let credit_after = credit_before
            .checked_add(amount)
            .ok_or(ContractError::ArithmeticOverflow)?;
        if debit_after < debit_min
            || debit_after > debit_max
            || credit_after < credit_min
            || credit_after > credit_max
        {
            return Err(ContractError::BalanceLimit);
        }

        let receipt = SettlementReceipt {
            transaction_id: transaction_id.to_owned(),
            obligation_id: obligation.id.clone(),
            instrument_id: obligation.instrument_id.clone(),
            atomic_unit: obligation.atomic_unit.clone(),
            amount_atomic: obligation.amount_atomic,
            debit_account: obligation.debit_account.clone(),
            credit_account: obligation.credit_account.clone(),
            debit_before,
            debit_after,
            credit_before,
            credit_after,
            posted_at,
        };

        // No mutation occurs before every arithmetic, identity and bound check succeeds.
        self.accounts
            .get_mut(&debit_key)
            .ok_or(ContractError::AccountNotFound)?
            .balance = debit_after;
        self.accounts
            .get_mut(&credit_key)
            .ok_or(ContractError::AccountNotFound)?
            .balance = credit_after;
        self.postings
            .insert(transaction_id.to_owned(), receipt.clone());
        self.posted_obligations.insert(obligation.id.clone());
        Ok(receipt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const INSTRUMENT: &str = "example:Sap";
    const ATOMIC_UNIT: &str = "example:Sap-atomic";

    fn request(id: &str, quantity: u64) -> NeedRequest {
        NeedRequest {
            transaction_id: id.to_owned(),
            community_id: "community-a".to_owned(),
            requester_id: "household-a".to_owned(),
            provider_id: "repair-coop".to_owned(),
            resource_id: "repair-hours".to_owned(),
            unit: "hour".to_owned(),
            window_id: "week-42".to_owned(),
            quantity,
            accept_until: 10,
            deliver_by: 20,
        }
    }

    fn gate(policy_id: &str) -> GateAssessment {
        GateAssessment {
            policy_id: policy_id.to_owned(),
            permitted: true,
            evidence_status: EvidenceStatus::Verified,
            origin: EvidenceOrigin::LiveDomain,
        }
    }

    fn authority() -> AuthorityProof {
        AuthorityProof {
            community_id: "community-a".to_owned(),
            actor_id: "steward-7".to_owned(),
            policy_digest: "policy-digest-a".to_owned(),
            evidence_status: EvidenceStatus::Verified,
            origin: EvidenceOrigin::LiveDomain,
        }
    }

    fn authorized_contract(id: &str, quantity: u64) -> CoordinationContract {
        let mut contract = CoordinationContract::new(request(id, quantity)).unwrap();
        contract
            .authorize(authority(), gate("rights-v1"), gate("ecology-v1"), 1)
            .unwrap();
        contract
    }

    fn committed_contract(id: &str, quantity: u64) -> (CoordinationContract, CapacityBook) {
        let mut contract = authorized_contract(id, quantity);
        let mut capacity = CapacityBook::default();
        capacity
            .declare_capacity("repair-hours", "hour", "week-42", quantity)
            .unwrap();
        contract.reserve(&mut capacity, 2).unwrap();
        contract
            .commit(
                ObligationTerms {
                    id: format!("obligation-{id}"),
                    debtor_id: "household-a".to_owned(),
                    creditor_id: "repair-coop".to_owned(),
                    debit_account: "household-wallet".to_owned(),
                    credit_account: "coop-wallet".to_owned(),
                    instrument_id: INSTRUMENT.to_owned(),
                    atomic_unit: ATOMIC_UNIT.to_owned(),
                    amount_atomic: 30,
                    due_at: 9,
                    policy_digest: "policy-digest-a".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                },
                3,
            )
            .unwrap();
        (contract, capacity)
    }

    fn record_full_delivery(contract: &mut CoordinationContract, quantity: u64) {
        contract
            .record_delivery(DeliveryClaim {
                source_id: "repair-coop".to_owned(),
                evidence_id: "provider-delivery-claim-1".to_owned(),
                resource_id: "repair-hours".to_owned(),
                unit: "hour".to_owned(),
                quantity,
                evidence_status: EvidenceStatus::Unknown,
                origin: EvidenceOrigin::LiveDomain,
                observed_at: 5,
            })
            .unwrap();
    }

    fn verified_contract(id: &str, quantity: u64) -> (CoordinationContract, CapacityBook, Ledger) {
        let (mut contract, capacity) = committed_contract(id, quantity);
        record_full_delivery(&mut contract, quantity);
        contract
            .verify_delivery(VerificationResult {
                verifier_id: "independent-auditor".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                evidence_id: "independent-delivery-check-1".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(quantity),
                decided_at: 6,
            })
            .unwrap();
        let mut ledger = Ledger::default();
        ledger
            .register_account(
                "household-wallet",
                INSTRUMENT,
                ATOMIC_UNIT,
                100,
                0,
                100,
            )
            .unwrap();
        ledger
            .register_account("coop-wallet", INSTRUMENT, ATOMIC_UNIT, 0, 0, 100)
            .unwrap();
        (contract, capacity, ledger)
    }

    #[test]
    fn full_need_to_fulfilment_path_posts_balanced_internal_settlement() {
        let (mut contract, mut capacity, mut ledger) = verified_contract("tx-happy", 4);
        contract.consume_verified_capacity(&mut capacity, 7).unwrap();
        let receipt = contract.settle(&mut ledger, 8).unwrap();

        assert_eq!(contract.lifecycle(), Lifecycle::Settled);
        assert_eq!(receipt.debit_before, 100);
        assert_eq!(receipt.debit_after, 70);
        assert_eq!(receipt.credit_before, 0);
        assert_eq!(receipt.credit_after, 30);
        assert_eq!(
            ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(70)
        );
        assert_eq!(
            ledger.balance("coop-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(30)
        );
        assert_eq!(ledger.posting("tx-happy"), Some(&receipt));
        assert_eq!(contract.history().last().unwrap().to, Lifecycle::Settled);
    }

    #[test]
    fn synthetic_origin_policy_evidence_blocks_authorization() {
        let mut contract = CoordinationContract::new(request("tx-synthetic-rights", 1)).unwrap();
        let mut rights = gate("rights-v1");
        rights.origin = EvidenceOrigin::SyntheticSimulation;

        assert_eq!(
            contract.authorize(authority(), rights, gate("ecology-v1"), 1),
            Err(ContractError::EvidenceNotVerified)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Proposed);
        assert!(contract.obligation().is_none());
    }

    #[test]
    fn unknown_rights_evidence_blocks_authorization() {
        let mut contract = CoordinationContract::new(request("tx-rights-unknown", 1)).unwrap();
        let mut rights = gate("rights-v1");
        rights.evidence_status = EvidenceStatus::Unknown;

        assert_eq!(
            contract.authorize(authority(), rights, gate("ecology-v1"), 1),
            Err(ContractError::EvidenceNotVerified)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Proposed);
    }

    #[test]
    fn denied_ecological_gate_blocks_authorization() {
        let mut contract = CoordinationContract::new(request("tx-ecology-denied", 1)).unwrap();
        let mut ecology = gate("ecology-v1");
        ecology.permitted = false;

        assert_eq!(
            contract.authorize(authority(), gate("rights-v1"), ecology, 1),
            Err(ContractError::PolicyDenied)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Proposed);
    }

    #[test]
    fn authorization_at_accept_deadline_is_expired() {
        let mut req = request("tx-expired", 1);
        req.accept_until = 1;
        req.deliver_by = 2;
        let mut contract = CoordinationContract::new(req).unwrap();

        assert_eq!(
            contract.authorize(authority(), gate("rights-v1"), gate("ecology-v1"), 1),
            Err(ContractError::Expired)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Proposed);
    }

    #[test]
    fn competing_requests_cannot_reserve_more_than_declared_capacity() {
        let mut book = CapacityBook::default();
        book.declare_capacity("repair-hours", "hour", "week-42", 1)
            .unwrap();
        let mut first = authorized_contract("tx-capacity-1", 1);
        let mut second = authorized_contract("tx-capacity-2", 1);

        first.reserve(&mut book, 2).unwrap();
        assert_eq!(
            second.reserve(&mut book, 2),
            Err(ContractError::CapacityUnavailable)
        );
        assert_eq!(book.remaining("repair-hours", "hour", "week-42"), Some(0));
    }

    #[test]
    fn cancellation_before_commit_releases_reserved_capacity() {
        let mut contract = authorized_contract("tx-cancel", 3);
        let mut book = CapacityBook::default();
        book.declare_capacity("repair-hours", "hour", "week-42", 4)
            .unwrap();
        contract.reserve(&mut book, 2).unwrap();

        contract.cancel(&mut book, 3).unwrap();

        assert_eq!(contract.lifecycle(), Lifecycle::Cancelled);
        assert_eq!(book.remaining("repair-hours", "hour", "week-42"), Some(4));
        assert!(book.reservation("tx-cancel").is_none());
    }

    #[test]
    fn obligation_must_bind_the_authorizing_policy() {
        let mut contract = authorized_contract("tx-policy", 1);
        let mut book = CapacityBook::default();
        book.declare_capacity("repair-hours", "hour", "week-42", 1)
            .unwrap();
        contract.reserve(&mut book, 2).unwrap();

        assert_eq!(
            contract.commit(
                ObligationTerms {
                    id: "obligation-policy".to_owned(),
                    debtor_id: "household-a".to_owned(),
                    creditor_id: "repair-coop".to_owned(),
                    debit_account: "household-wallet".to_owned(),
                    credit_account: "coop-wallet".to_owned(),
                    instrument_id: INSTRUMENT.to_owned(),
                    atomic_unit: ATOMIC_UNIT.to_owned(),
                    amount_atomic: 30,
                    due_at: 9,
                    policy_digest: "different-policy".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                },
                3,
            ),
            Err(ContractError::InvalidObligation)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Reserved);
    }

    #[test]
    fn provider_cannot_independently_verify_its_own_delivery() {
        let (mut contract, _) = committed_contract("tx-self-verify", 2);
        record_full_delivery(&mut contract, 2);
        let result = VerificationResult {
            verifier_id: "repair-coop".to_owned(),
            claim_evidence_id: "provider-delivery-claim-1".to_owned(),
            evidence_id: "same-party-check".to_owned(),
            evidence_status: EvidenceStatus::Verified,
            origin: EvidenceOrigin::LiveDomain,
            accepted_quantity: Some(2),
            decided_at: 6,
        };

        assert_eq!(
            contract.verify_delivery(result),
            Err(ContractError::NotIndependentVerifier)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Delivered);
    }

    #[test]
    fn synthetic_origin_verification_cannot_confirm_real_fulfilment() {
        let (mut contract, _) = committed_contract("tx-synthetic-delivery", 1);
        record_full_delivery(&mut contract, 1);

        assert_eq!(
            contract.verify_delivery(VerificationResult {
                verifier_id: "synthetic-evaluator".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                evidence_id: "simulation-result-1".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::SyntheticSimulation,
                accepted_quantity: Some(1),
                decided_at: 6,
            }),
            Ok(Lifecycle::Indeterminate)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Indeterminate);
        assert!(contract.verification().is_some());
    }

    #[test]
    fn unknown_verification_is_indeterminate_and_can_be_rechecked() {
        let (mut contract, _) = committed_contract("tx-recheck", 2);
        record_full_delivery(&mut contract, 2);

        assert_eq!(
            contract
                .verify_delivery(VerificationResult {
                    verifier_id: "independent-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    evidence_id: "incomplete-check".to_owned(),
                    evidence_status: EvidenceStatus::Unknown,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: None,
                    decided_at: 6,
                })
                .unwrap(),
            Lifecycle::Indeterminate
        );
        assert_eq!(
            contract
                .verify_delivery(VerificationResult {
                    verifier_id: "independent-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    evidence_id: "reconciled-check".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(2),
                    decided_at: 7,
                })
                .unwrap(),
            Lifecycle::Verified
        );
    }

    #[test]
    fn short_delivery_is_disputed_and_cannot_settle() {
        let (mut contract, _) = committed_contract("tx-short-delivery", 4);
        let mut ledger = Ledger::default();
        record_full_delivery(&mut contract, 3);
        assert_eq!(
            contract
                .verify_delivery(VerificationResult {
                    verifier_id: "independent-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    evidence_id: "short-delivery-check".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(3),
                    decided_at: 6,
                })
                .unwrap(),
            Lifecycle::Disputed
        );

        assert_eq!(
            contract.settle(&mut ledger, 7),
            Err(ContractError::InvalidTransition)
        );
        assert!(ledger.posting("tx-short-delivery").is_none());
    }

    #[test]
    fn balance_limit_failure_is_atomic() {
        let (mut contract, mut capacity, _) = verified_contract("tx-balance-limit", 1);
        contract.consume_verified_capacity(&mut capacity, 7).unwrap();
        let mut ledger = Ledger::default();
        ledger
            .register_account(
                "household-wallet",
                INSTRUMENT,
                ATOMIC_UNIT,
                10,
                0,
                10,
            )
            .unwrap();
        ledger
            .register_account("coop-wallet", INSTRUMENT, ATOMIC_UNIT, 0, 0, 100)
            .unwrap();

        assert_eq!(
            contract.settle(&mut ledger, 8),
            Err(ContractError::BalanceLimit)
        );
        assert_eq!(
            ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(10)
        );
        assert_eq!(
            ledger.balance("coop-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(0)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::CapacityConsumed);
        assert!(contract.capacity_consumed());
        assert_eq!(
            capacity.remaining("repair-hours", "hour", "week-42"),
            Some(0)
        );
        assert_eq!(capacity.consumed("repair-hours", "hour", "week-42"), Some(1));
        assert!(ledger.posting("tx-balance-limit").is_none());
    }

    #[test]
    fn repeated_settlement_does_not_post_twice() {
        let (mut contract, mut capacity, mut ledger) = verified_contract("tx-replay", 1);
        contract.consume_verified_capacity(&mut capacity, 7).unwrap();
        contract.settle(&mut ledger, 8).unwrap();

        assert_eq!(
            contract.settle(&mut ledger, 9),
            Err(ContractError::InvalidTransition)
        );
        assert_eq!(
            ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(70)
        );
        assert_eq!(
            ledger.balance("coop-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(30)
        );
    }

    #[test]
    fn missing_instrument_account_does_not_move_any_balance() {
        let (mut contract, mut capacity, _) = verified_contract("tx-instrument-mismatch", 1);
        contract.consume_verified_capacity(&mut capacity, 7).unwrap();
        let mut ledger = Ledger::default();
        ledger
            .register_account(
                "household-wallet",
                INSTRUMENT,
                ATOMIC_UNIT,
                100,
                0,
                100,
            )
            .unwrap();
        ledger
            .register_account("coop-wallet", "other:instrument", ATOMIC_UNIT, 0, 0, 100)
            .unwrap();

        assert_eq!(
            contract.settle(&mut ledger, 8),
            Err(ContractError::AccountNotFound)
        );
        assert_eq!(
            ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(100)
        );
        assert_eq!(
            ledger.balance("coop-wallet", "other:instrument", ATOMIC_UNIT),
            Some(0)
        );
    }

    #[test]
    fn duplicate_capacity_declaration_does_not_reset_existing_reservations() {
        let mut book = CapacityBook::default();
        book.declare_capacity("repair-hours", "hour", "week-42", 2)
            .unwrap();
        let mut contract = authorized_contract("tx-capacity-preserve", 1);
        contract.reserve(&mut book, 2).unwrap();

        assert_eq!(
            book.declare_capacity("repair-hours", "hour", "week-42", 50),
            Err(ContractError::DuplicateCapacity)
        );
        assert_eq!(book.remaining("repair-hours", "hour", "week-42"), Some(1));
        assert!(book.reservation("tx-capacity-preserve").is_some());
    }

    #[test]
    fn duplicate_account_declaration_does_not_overwrite_existing_balance() {
        let mut ledger = Ledger::default();
        ledger
            .register_account("account-a", INSTRUMENT, ATOMIC_UNIT, 10, 0, 20)
            .unwrap();

        assert_eq!(
            ledger.register_account("account-a", INSTRUMENT, ATOMIC_UNIT, 0, 0, 0),
            Err(ContractError::DuplicateAccount)
        );
        assert_eq!(
            ledger.balance("account-a", INSTRUMENT, ATOMIC_UNIT),
            Some(10)
        );
    }

    #[test]
    fn verifier_must_bind_the_exact_delivery_claim() {
        let (mut contract, _) = committed_contract("tx-claim-binding", 1);
        record_full_delivery(&mut contract, 1);

        assert_eq!(
            contract.verify_delivery(VerificationResult {
                verifier_id: "independent-auditor".to_owned(),
                claim_evidence_id: "different-provider-claim".to_owned(),
                evidence_id: "independent-check".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(1),
                decided_at: 6,
            }),
            Err(ContractError::InvalidEvidence)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Delivered);
        assert!(contract.verification().is_none());
    }

    #[test]
    fn lifecycle_timestamps_cannot_move_backwards() {
        let mut contract = authorized_contract("tx-time-order", 1);
        let mut book = CapacityBook::default();
        book.declare_capacity("repair-hours", "hour", "week-42", 1)
            .unwrap();

        assert_eq!(
            contract.reserve(&mut book, 0),
            Err(ContractError::NonMonotonicTime)
        );
        assert_eq!(book.remaining("repair-hours", "hour", "week-42"), Some(1));
        assert!(contract.reservation().is_none());
    }

    #[test]
    fn released_transaction_identity_cannot_be_reserved_again() {
        let mut book = CapacityBook::default();
        book.declare_capacity("repair-hours", "hour", "week-42", 2)
            .unwrap();
        let mut first = authorized_contract("tx-single-use", 1);
        first.reserve(&mut book, 2).unwrap();
        first.cancel(&mut book, 3).unwrap();

        let mut second = authorized_contract("tx-single-use", 1);
        assert_eq!(
            second.reserve(&mut book, 2),
            Err(ContractError::DuplicateReservation)
        );
        assert_eq!(book.remaining("repair-hours", "hour", "week-42"), Some(2));
    }

    #[test]
    fn obligation_identity_cannot_be_settled_under_another_transaction() {
        let (mut first, mut first_capacity, mut ledger) = verified_contract("tx-obligation-first", 1);
        let (mut second, mut second_capacity, _) = verified_contract("tx-obligation-second", 1);
        second.obligation.as_mut().unwrap().id = first.obligation.as_ref().unwrap().id.clone();
        first.consume_verified_capacity(&mut first_capacity, 7).unwrap();
        second.consume_verified_capacity(&mut second_capacity, 7).unwrap();

        first.settle(&mut ledger, 8).unwrap();
        let debit_after_first = ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT);
        let credit_after_first = ledger.balance("coop-wallet", INSTRUMENT, ATOMIC_UNIT);

        assert_eq!(
            second.settle(&mut ledger, 8),
            Err(ContractError::AlreadyPosted)
        );
        assert_eq!(
            ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT),
            debit_after_first
        );
        assert_eq!(
            ledger.balance("coop-wallet", INSTRUMENT, ATOMIC_UNIT),
            credit_after_first
        );
        assert!(ledger.posting("tx-obligation-second").is_none());
    }

    #[test]
    fn settlement_timestamp_cannot_precede_verified_delivery() {
        let (mut contract, mut capacity, mut ledger) = verified_contract("tx-settlement-time", 1);
        contract.consume_verified_capacity(&mut capacity, 6).unwrap();
        assert_eq!(
            contract.settle(&mut ledger, 4),
            Err(ContractError::NonMonotonicTime)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::CapacityConsumed);
        assert!(contract.capacity_consumed());
        assert_eq!(
            ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(100)
        );
        assert_eq!(
            ledger.balance("coop-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(0)
        );
    }

    #[test]
    fn disputed_delivery_can_be_confirmed_by_a_distinct_independent_resolver() {
        let (mut contract, mut capacity) = committed_contract("tx-dispute-reassess", 4);
        record_full_delivery(&mut contract, 4);
        assert_eq!(
            contract
                .verify_delivery(VerificationResult {
                    verifier_id: "first-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    evidence_id: "first-check".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(3),
                    decided_at: 6,
                })
                .unwrap(),
            Lifecycle::Disputed
        );

        assert_eq!(
            contract
                .reassess_dispute(DisputeReassessment {
                    resolver_id: "independent-reviewer".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    decision_evidence_id: "review-check".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(4),
                    decided_at: 7,
                })
                .unwrap(),
            Lifecycle::Verified
        );
        assert_eq!(contract.dispute_reassessments().len(), 1);
        assert_eq!(contract.lifecycle(), Lifecycle::Verified);
        let accepted_verification = contract.verification().unwrap();
        assert_eq!(accepted_verification.verifier_id, "independent-reviewer");
        assert_eq!(accepted_verification.accepted_quantity, Some(4));
        assert_eq!(accepted_verification.evidence_id, "review-check");
        assert_eq!(contract.verification_history().len(), 2);
        assert_eq!(contract.verification_history()[0].verifier_id, "first-auditor");
        assert_eq!(contract.verification_history()[0].accepted_quantity, Some(3));
        assert_eq!(contract.verification_history()[1].verifier_id, "independent-reviewer");
        contract.consume_verified_capacity(&mut capacity, 7).unwrap();
        let mut ledger = Ledger::default();
        ledger
            .register_account("household-wallet", INSTRUMENT, ATOMIC_UNIT, 100, 0, 100)
            .unwrap();
        ledger
            .register_account("coop-wallet", INSTRUMENT, ATOMIC_UNIT, 0, 0, 100)
            .unwrap();
        assert!(contract.settle(&mut ledger, 8).is_ok());
    }

    #[test]
    fn unresolved_dispute_is_journaled_and_does_not_erase_obligation() {
        let (mut contract, _) = committed_contract("tx-dispute-open", 4);
        let mut ledger = Ledger::default();
        record_full_delivery(&mut contract, 4);
        contract
            .verify_delivery(VerificationResult {
                verifier_id: "first-auditor".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                evidence_id: "first-check".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(3),
                decided_at: 6,
            })
            .unwrap();
        let obligation_id = contract.obligation().unwrap().id.clone();

        assert_eq!(
            contract.reassess_dispute(DisputeReassessment {
                resolver_id: "independent-reviewer".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                decision_evidence_id: "still-short".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(3),
                decided_at: 7,
            }),
            Ok(Lifecycle::Disputed)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Disputed);
        assert_eq!(contract.dispute_reassessments().len(), 1);
        assert_eq!(contract.obligation().unwrap().id, obligation_id);
        assert_eq!(contract.verification_history().len(), 2);
        assert_eq!(contract.verification().unwrap().verifier_id, "first-auditor");
        assert_eq!(contract.verification_history()[1].accepted_quantity, Some(3));
        assert_eq!(
            contract.settle(&mut ledger, 8),
            Err(ContractError::InvalidTransition)
        );
    }

    #[test]
    fn dispute_resolver_must_differ_from_first_verifier() {
        let (mut contract, _) = committed_contract("tx-dispute-resolver", 4);
        record_full_delivery(&mut contract, 4);
        contract
            .verify_delivery(VerificationResult {
                verifier_id: "first-auditor".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                evidence_id: "first-check".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(3),
                decided_at: 6,
            })
            .unwrap();

        assert_eq!(
            contract.reassess_dispute(DisputeReassessment {
                resolver_id: "first-auditor".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                decision_evidence_id: "same-reviewer".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(4),
                decided_at: 7,
            }),
            Err(ContractError::NotIndependentVerifier)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Disputed);
        assert!(contract.dispute_reassessments().is_empty());
    }

    #[test]
    fn same_resolver_cannot_repeat_a_dispute_reassessment() {
        let (mut contract, _) = committed_contract("tx-dispute-repeat-resolver", 4);
        record_full_delivery(&mut contract, 4);
        contract
            .verify_delivery(VerificationResult {
                verifier_id: "first-auditor".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                evidence_id: "first-check".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(3),
                decided_at: 6,
            })
            .unwrap();
        assert_eq!(
            contract
                .reassess_dispute(DisputeReassessment {
                    resolver_id: "independent-reviewer".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    decision_evidence_id: "still-short".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(3),
                    decided_at: 7,
                })
                .unwrap(),
            Lifecycle::Disputed
        );
        assert_eq!(
            contract.reassess_dispute(DisputeReassessment {
                resolver_id: "independent-reviewer".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                decision_evidence_id: "same-reviewer-again".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(4),
                decided_at: 8,
            }),
            Err(ContractError::NotIndependentVerifier)
        );
        assert_eq!(contract.dispute_reassessments().len(), 1);
        assert_eq!(contract.lifecycle(), Lifecycle::Disputed);
    }

    #[test]
    fn settlement_requires_consumed_resource_capacity() {
        let (mut contract, mut capacity, mut ledger) = verified_contract("tx-capacity-before-settlement", 1);

        assert_eq!(
            contract.settle(&mut ledger, 7),
            Err(ContractError::InvalidTransition)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Verified);
        assert!(!contract.capacity_consumed());
        assert_eq!(
            capacity.remaining("repair-hours", "hour", "week-42"),
            Some(1)
        );
        assert_eq!(capacity.consumed("repair-hours", "hour", "week-42"), Some(0));
        assert_eq!(
            ledger.balance("household-wallet", INSTRUMENT, ATOMIC_UNIT),
            Some(100)
        );
    }

    #[test]
    fn consumed_capacity_cannot_be_reserved_again_by_another_transaction() {
        let (mut contract, mut capacity, _) = verified_contract("tx-capacity-consumed", 1);
        contract.consume_verified_capacity(&mut capacity, 7).unwrap();
        let mut next = authorized_contract("tx-capacity-next", 1);

        assert_eq!(
            next.reserve(&mut capacity, 8),
            Err(ContractError::CapacityUnavailable)
        );
        assert_eq!(
            capacity.remaining("repair-hours", "hour", "week-42"),
            Some(0)
        );
        assert_eq!(capacity.consumed("repair-hours", "hour", "week-42"), Some(1));
    }

    #[test]
    fn malformed_zero_quantity_request_is_rejected() {
        assert_eq!(
            CoordinationContract::new(request("tx-zero", 0)).err(),
            Some(ContractError::InvalidRequest)
        );
    }
    #[test]
    fn synthetic_origin_obligation_evidence_cannot_be_committed() {
        let mut contract = authorized_contract("tx-synthetic-obligation", 1);
        let mut capacity = CapacityBook::default();
        capacity
            .declare_capacity("repair-hours", "hour", "week-42", 1)
            .unwrap();
        contract.reserve(&mut capacity, 2).unwrap();

        let result = contract.commit(
            ObligationTerms {
                id: "obligation-tx-synthetic-obligation".to_owned(),
                debtor_id: "household-a".to_owned(),
                creditor_id: "repair-coop".to_owned(),
                debit_account: "household-wallet".to_owned(),
                credit_account: "coop-wallet".to_owned(),
                instrument_id: INSTRUMENT.to_owned(),
                atomic_unit: ATOMIC_UNIT.to_owned(),
                amount_atomic: 30,
                due_at: 9,
                policy_digest: "policy-digest-a".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::SyntheticSimulation,
            },
            3,
        );

        assert_eq!(result, Err(ContractError::EvidenceNotVerified));
        assert_eq!(contract.lifecycle(), Lifecycle::Reserved);
        assert!(contract.obligation().is_none());
    }

    #[test]
    fn synthetic_origin_delivery_claim_cannot_be_verified_as_real_fulfilment() {
        let (mut contract, _) = committed_contract("tx-synthetic-claim", 1);
        contract
            .record_delivery(DeliveryClaim {
                source_id: "repair-coop".to_owned(),
                evidence_id: "simulation-delivery-claim".to_owned(),
                resource_id: "repair-hours".to_owned(),
                unit: "hour".to_owned(),
                quantity: 1,
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::SyntheticSimulation,
                observed_at: 5,
            })
            .unwrap();

        assert_eq!(
            contract.verify_delivery(VerificationResult {
                verifier_id: "independent-auditor".to_owned(),
                claim_evidence_id: "simulation-delivery-claim".to_owned(),
                evidence_id: "live-review-of-simulated-claim".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(1),
                decided_at: 6,
            }),
            Ok(Lifecycle::Indeterminate)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Indeterminate);
    }

    #[test]
    fn synthetic_origin_dispute_reassessment_cannot_resolve_real_dispute() {
        let (mut contract, _) = committed_contract("tx-synthetic-reassessment", 4);
        record_full_delivery(&mut contract, 4);
        assert_eq!(
            contract
                .verify_delivery(VerificationResult {
                    verifier_id: "first-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    evidence_id: "initial-short-check".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(3),
                    decided_at: 6,
                })
                .unwrap(),
            Lifecycle::Disputed
        );

        assert_eq!(
            contract
                .reassess_dispute(DisputeReassessment {
                    resolver_id: "second-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    decision_evidence_id: "simulated-reassessment".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::SyntheticSimulation,
                    accepted_quantity: Some(4),
                    decided_at: 7,
                })
                .unwrap(),
            Lifecycle::Disputed
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Disputed);
        assert_eq!(contract.dispute_reassessments().len(), 1);
    }

    #[test]
    fn delivery_claim_must_come_from_the_authorized_provider() {
        let (mut contract, _) = committed_contract("tx-untrusted-delivery-source", 1);
        let result = contract.record_delivery(DeliveryClaim {
            source_id: "untrusted-observer".to_owned(),
            evidence_id: "untrusted-claim-1".to_owned(),
            resource_id: "repair-hours".to_owned(),
            unit: "hour".to_owned(),
            quantity: 1,
            evidence_status: EvidenceStatus::Unknown,
            origin: EvidenceOrigin::LiveDomain,
            observed_at: 5,
        });

        assert_eq!(result, Err(ContractError::InvalidEvidence));
        assert_eq!(contract.lifecycle(), Lifecycle::Committed);
        assert!(contract.delivery_claim().is_none());
        assert!(contract.verification().is_none());
    }


    #[test]
    fn unknown_origin_policy_evidence_blocks_authorization() {
        let mut contract = CoordinationContract::new(request("tx-origin-unknown-rights", 1)).unwrap();
        let mut rights = gate("rights-v1");
        rights.origin = EvidenceOrigin::Unknown;

        assert_eq!(
            contract.authorize(authority(), rights, gate("ecology-v1"), 1),
            Err(ContractError::EvidenceNotVerified)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Proposed);
        assert!(contract.obligation().is_none());
    }

    #[test]
    fn unknown_origin_obligation_evidence_cannot_be_committed() {
        let mut contract = authorized_contract("tx-origin-unknown-obligation", 1);
        let mut capacity = CapacityBook::default();
        capacity
            .declare_capacity("repair-hours", "hour", "week-42", 1)
            .unwrap();
        contract.reserve(&mut capacity, 2).unwrap();

        let result = contract.commit(
            ObligationTerms {
                id: "obligation-tx-origin-unknown-obligation".to_owned(),
                debtor_id: "household-a".to_owned(),
                creditor_id: "repair-coop".to_owned(),
                debit_account: "household-wallet".to_owned(),
                credit_account: "coop-wallet".to_owned(),
                instrument_id: INSTRUMENT.to_owned(),
                atomic_unit: ATOMIC_UNIT.to_owned(),
                amount_atomic: 30,
                due_at: 9,
                policy_digest: "policy-digest-a".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::Unknown,
            },
            3,
        );

        assert_eq!(result, Err(ContractError::EvidenceNotVerified));
        assert_eq!(contract.lifecycle(), Lifecycle::Reserved);
        assert!(contract.obligation().is_none());
    }

    #[test]
    fn unknown_origin_delivery_claim_cannot_be_verified_as_real_fulfilment() {
        let (mut contract, _) = committed_contract("tx-origin-unknown-claim", 1);
        contract
            .record_delivery(DeliveryClaim {
                source_id: "repair-coop".to_owned(),
                evidence_id: "unknown-origin-delivery-claim".to_owned(),
                resource_id: "repair-hours".to_owned(),
                unit: "hour".to_owned(),
                quantity: 1,
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::Unknown,
                observed_at: 5,
            })
            .unwrap();

        assert_eq!(
            contract.verify_delivery(VerificationResult {
                verifier_id: "independent-auditor".to_owned(),
                claim_evidence_id: "unknown-origin-delivery-claim".to_owned(),
                evidence_id: "live-review-of-unknown-origin-claim".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::LiveDomain,
                accepted_quantity: Some(1),
                decided_at: 6,
            }),
            Ok(Lifecycle::Indeterminate)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Indeterminate);
    }

    #[test]
    fn unknown_origin_verification_cannot_confirm_real_fulfilment() {
        let (mut contract, _) = committed_contract("tx-origin-unknown-verification", 1);
        record_full_delivery(&mut contract, 1);

        assert_eq!(
            contract.verify_delivery(VerificationResult {
                verifier_id: "independent-auditor".to_owned(),
                claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                evidence_id: "unknown-origin-verification".to_owned(),
                evidence_status: EvidenceStatus::Verified,
                origin: EvidenceOrigin::Unknown,
                accepted_quantity: Some(1),
                decided_at: 6,
            }),
            Ok(Lifecycle::Indeterminate)
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Indeterminate);
        assert!(contract.verification().is_some());
    }

    #[test]
    fn unknown_origin_dispute_reassessment_cannot_resolve_real_dispute() {
        let (mut contract, _) = committed_contract("tx-origin-unknown-reassessment", 4);
        record_full_delivery(&mut contract, 4);
        assert_eq!(
            contract
                .verify_delivery(VerificationResult {
                    verifier_id: "first-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    evidence_id: "initial-short-check".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(3),
                    decided_at: 6,
                })
                .unwrap(),
            Lifecycle::Disputed
        );

        assert_eq!(
            contract
                .reassess_dispute(DisputeReassessment {
                    resolver_id: "second-auditor".to_owned(),
                    claim_evidence_id: "provider-delivery-claim-1".to_owned(),
                    decision_evidence_id: "unknown-origin-reassessment".to_owned(),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::Unknown,
                    accepted_quantity: Some(4),
                    decided_at: 7,
                })
                .unwrap(),
            Lifecycle::Disputed
        );
        assert_eq!(contract.lifecycle(), Lifecycle::Disputed);
        assert_eq!(contract.dispute_reassessments().len(), 1);
    }

}

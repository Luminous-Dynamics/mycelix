//! Append-only reference ledger for staged/partial physical fulfilment.
//!
//! This model is intentionally separate from the v1 single-claim lifecycle.
//! It records tranches and acceptance decisions, but does not authorize money
//! movement or consume the existing CapacityBook. It is a research contract for
//! future integration, not a production zome or distributed-consensus protocol.

use super::{EvidenceOrigin, EvidenceStatus};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrancheError {
    InvalidRequest,
    InvalidClaim,
    InvalidVerification,
    DuplicateClaim,
    DuplicateEvidence,
    OverCommitment,
    OverFulfillment,
    MissingClaim,
    ClaimAlreadyFinal,
    NotIndependentVerifier,
    DeadlinePassed,
    NonMonotonicJournalTime,
    ArithmeticOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FulfillmentRequest {
    pub request_id: String,
    pub requester_id: String,
    pub provider_id: String,
    pub resource_id: String,
    pub unit: String,
    pub requested_quantity: u64,
    pub deliver_by: u64,
}

impl FulfillmentRequest {
    fn validate(&self) -> Result<(), TrancheError> {
        if [
            self.request_id.as_str(),
            self.requester_id.as_str(),
            self.provider_id.as_str(),
            self.resource_id.as_str(),
            self.unit.as_str(),
        ]
        .iter()
        .any(|value| value.trim().is_empty())
            || self.requester_id == self.provider_id
            || self.requested_quantity == 0
            || self.deliver_by == 0
        {
            return Err(TrancheError::InvalidRequest);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeliveryTrancheClaim {
    pub claim_id: String,
    pub source_id: String,
    pub evidence_id: String,
    pub resource_id: String,
    pub unit: String,
    pub quantity: u64,
    pub origin: EvidenceOrigin,
    /// Time the provider says the tranche was physically delivered.
    pub observed_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrancheVerification {
    pub verifier_id: String,
    pub claim_id: String,
    pub claim_evidence_id: String,
    pub evidence_id: String,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
    /// Quantity independently accepted for this exact claim. None is indeterminate.
    pub accepted_quantity: Option<u64>,
    pub decided_at: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrancheState {
    Claimed,
    Indeterminate,
    Disputed,
    PartiallyAccepted,
    Accepted,
}

impl TrancheState {
    fn reserves_full_claim(self) -> bool {
        matches!(self, Self::Claimed | Self::Indeterminate | Self::Disputed)
    }

    fn is_final(self) -> bool {
        matches!(self, Self::PartiallyAccepted | Self::Accepted)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrancheRecord {
    pub claim: DeliveryTrancheClaim,
    pub state: TrancheState,
    pub accepted_quantity: u64,
    pub verification_history: Vec<TrancheVerification>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TrancheJournalEntry {
    ClaimRecorded {
        claim: DeliveryTrancheClaim,
        recorded_at: u64,
    },
    VerificationRecorded(TrancheVerification),
}

/// Serial, append-only fulfilment ledger for one requested resource quantity.
///
/// Pending, indeterminate, and disputed claims reserve their claimed quantity.
/// A verified partial acceptance counts only the accepted quantity and releases
/// the rejected remainder for a later tranche. Unverified or synthetic evidence
/// never increments accepted quantity. This ledger is not a settlement engine.
#[derive(Clone, Debug)]
pub struct FulfillmentLedger {
    request: FulfillmentRequest,
    tranches: BTreeMap<String, TrancheRecord>,
    seen_evidence_ids: BTreeSet<String>,
    journal: Vec<TrancheJournalEntry>,
    last_recorded_at: Option<u64>,
}

impl FulfillmentLedger {
    pub fn new(request: FulfillmentRequest) -> Result<Self, TrancheError> {
        request.validate()?;
        Ok(Self {
            request,
            tranches: BTreeMap::new(),
            seen_evidence_ids: BTreeSet::new(),
            journal: Vec::new(),
            last_recorded_at: None,
        })
    }

    pub fn request(&self) -> &FulfillmentRequest {
        &self.request
    }

    pub fn journal(&self) -> &[TrancheJournalEntry] {
        &self.journal
    }

    pub fn tranche(&self, claim_id: &str) -> Option<&TrancheRecord> {
        self.tranches.get(claim_id)
    }

    pub fn accepted_quantity(&self) -> Result<u64, TrancheError> {
        self.tranches.values().try_fold(0_u64, |total, record| {
            total
                .checked_add(record.accepted_quantity)
                .ok_or(TrancheError::ArithmeticOverflow)
        })
    }

    pub fn remaining_quantity(&self) -> Result<u64, TrancheError> {
        self.request
            .requested_quantity
            .checked_sub(self.accepted_quantity()?)
            .ok_or(TrancheError::OverFulfillment)
    }

    pub fn is_fully_fulfilled(&self) -> Result<bool, TrancheError> {
        Ok(self.remaining_quantity()? == 0
            && self
                .tranches
                .values()
                .all(|record| !record.state.reserves_full_claim()))
    }

    fn reserved_or_accepted_quantity(&self) -> Result<u64, TrancheError> {
        self.tranches.values().try_fold(self.accepted_quantity()?, |total, record| {
            if record.state.reserves_full_claim() {
                total
                    .checked_add(record.claim.quantity)
                    .ok_or(TrancheError::ArithmeticOverflow)
            } else {
                Ok(total)
            }
        })
    }

    fn ensure_journal_time(&self, at: u64) -> Result<(), TrancheError> {
        if self.last_recorded_at.is_some_and(|last| at < last) {
            return Err(TrancheError::NonMonotonicJournalTime);
        }
        Ok(())
    }

    /// Record a provider's tranche assertion. This is not proof of delivery.
    pub fn record_claim(
        &mut self,
        claim: DeliveryTrancheClaim,
        recorded_at: u64,
    ) -> Result<(), TrancheError> {
        self.ensure_journal_time(recorded_at)?;
        if claim.claim_id.trim().is_empty()
            || claim.source_id != self.request.provider_id
            || claim.source_id.trim().is_empty()
            || claim.evidence_id.trim().is_empty()
            || claim.resource_id != self.request.resource_id
            || claim.unit != self.request.unit
            || claim.quantity == 0
            || claim.observed_at > recorded_at
        {
            return Err(TrancheError::InvalidClaim);
        }
        if claim.observed_at > self.request.deliver_by {
            return Err(TrancheError::DeadlinePassed);
        }
        if self.tranches.contains_key(&claim.claim_id) {
            return Err(TrancheError::DuplicateClaim);
        }
        if self.seen_evidence_ids.contains(&claim.evidence_id) {
            return Err(TrancheError::DuplicateEvidence);
        }
        let allocated = self
            .reserved_or_accepted_quantity()?
            .checked_add(claim.quantity)
            .ok_or(TrancheError::ArithmeticOverflow)?;
        if allocated > self.request.requested_quantity {
            return Err(TrancheError::OverCommitment);
        }

        let claim_id = claim.claim_id.clone();
        self.seen_evidence_ids.insert(claim.evidence_id.clone());
        self.tranches.insert(
            claim_id,
            TrancheRecord {
                claim: claim.clone(),
                state: TrancheState::Claimed,
                accepted_quantity: 0,
                verification_history: Vec::new(),
            },
        );
        self.journal.push(TrancheJournalEntry::ClaimRecorded {
            claim,
            recorded_at,
        });
        self.last_recorded_at = Some(recorded_at);
        Ok(())
    }

    /// Verify a claim or retry an indeterminate/disputed claim using fresh evidence.
    ///
    /// A disputed claim requires a resolver distinct from the prior verifier.
    /// Every accepted decision is journaled; a final tranche cannot be counted twice.
    pub fn verify_claim(
        &mut self,
        result: TrancheVerification,
    ) -> Result<TrancheState, TrancheError> {
        self.ensure_journal_time(result.decided_at)?;
        let record = self
            .tranches
            .get(&result.claim_id)
            .ok_or(TrancheError::MissingClaim)?;
        if record.state.is_final() {
            return Err(TrancheError::ClaimAlreadyFinal);
        }
        if result.claim_evidence_id != record.claim.evidence_id
            || result.verifier_id.trim().is_empty()
            || result.evidence_id.trim().is_empty()
        {
            return Err(TrancheError::InvalidVerification);
        }
        if self.seen_evidence_ids.contains(&result.evidence_id) {
            return Err(TrancheError::DuplicateEvidence);
        }
        if result.verifier_id == self.request.requester_id
            || result.verifier_id == self.request.provider_id
            || result.verifier_id == record.claim.source_id
        {
            return Err(TrancheError::NotIndependentVerifier);
        }
        if result.decided_at < record.claim.observed_at {
            return Err(TrancheError::NonMonotonicJournalTime);
        }
        if record.state == TrancheState::Disputed
            && record
                .verification_history
                .last()
                .is_some_and(|previous| previous.verifier_id == result.verifier_id)
        {
            return Err(TrancheError::NotIndependentVerifier);
        }

        let next = if result.evidence_status != EvidenceStatus::Verified
            || result.origin != EvidenceOrigin::LiveDomain
            || record.claim.origin != EvidenceOrigin::LiveDomain
            || result.accepted_quantity.is_none()
        {
            TrancheState::Indeterminate
        } else {
            let accepted = result.accepted_quantity.ok_or(TrancheError::InvalidVerification)?;
            if accepted > record.claim.quantity {
                return Err(TrancheError::OverFulfillment);
            }
            if accepted == 0 {
                TrancheState::Disputed
            } else if accepted < record.claim.quantity {
                TrancheState::PartiallyAccepted
            } else {
                TrancheState::Accepted
            }
        };

        let accepted = if next.is_final() {
            result.accepted_quantity.ok_or(TrancheError::InvalidVerification)?
        } else {
            0
        };
        let accepted_without_this = self
            .accepted_quantity()?
            .checked_sub(record.accepted_quantity)
            .ok_or(TrancheError::ArithmeticOverflow)?;
        if accepted_without_this
            .checked_add(accepted)
            .ok_or(TrancheError::ArithmeticOverflow)?
            > self.request.requested_quantity
        {
            return Err(TrancheError::OverFulfillment);
        }

        let record = self
            .tranches
            .get_mut(&result.claim_id)
            .ok_or(TrancheError::MissingClaim)?;
        record.state = next;
        record.accepted_quantity = accepted;
        record.verification_history.push(result.clone());
        self.seen_evidence_ids.insert(result.evidence_id.clone());
        self.journal
            .push(TrancheJournalEntry::VerificationRecorded(result.clone()));
        self.last_recorded_at = Some(result.decided_at);
        Ok(next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ledger(quantity: u64) -> FulfillmentLedger {
        FulfillmentLedger::new(FulfillmentRequest {
            request_id: "need-001".to_owned(),
            requester_id: "household-a".to_owned(),
            provider_id: "repair-coop".to_owned(),
            resource_id: "repair-hours".to_owned(),
            unit: "hour".to_owned(),
            requested_quantity: quantity,
            deliver_by: 50,
        })
        .unwrap()
    }

    fn claim(id: &str, quantity: u64, observed_at: u64) -> DeliveryTrancheClaim {
        DeliveryTrancheClaim {
            claim_id: id.to_owned(),
            source_id: "repair-coop".to_owned(),
            evidence_id: format!("claim-evidence-{id}"),
            resource_id: "repair-hours".to_owned(),
            unit: "hour".to_owned(),
            quantity,
            origin: EvidenceOrigin::LiveDomain,
            observed_at,
        }
    }

    fn verification(
        claim: &DeliveryTrancheClaim,
        evidence_id: &str,
        verifier_id: &str,
        status: EvidenceStatus,
        origin: EvidenceOrigin,
        accepted_quantity: Option<u64>,
        decided_at: u64,
    ) -> TrancheVerification {
        TrancheVerification {
            verifier_id: verifier_id.to_owned(),
            claim_id: claim.claim_id.clone(),
            claim_evidence_id: claim.evidence_id.clone(),
            evidence_id: evidence_id.to_owned(),
            evidence_status: status,
            origin,
            accepted_quantity,
            decided_at,
        }
    }

    #[test]
    fn multiple_partial_tranches_can_fulfil_one_request_exactly() {
        let mut book = ledger(10);
        let first = claim("tranche-1", 4, 2);
        book.record_claim(first.clone(), 3).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &first, "verify-1", "auditor-1", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(4), 4
            )).unwrap(),
            TrancheState::Accepted
        );
        let second = claim("tranche-2", 6, 5);
        book.record_claim(second.clone(), 6).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &second, "verify-2", "auditor-2", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(6), 7
            )).unwrap(),
            TrancheState::Accepted
        );
        assert_eq!(book.accepted_quantity(), Ok(10));
        assert_eq!(book.remaining_quantity(), Ok(0));
        assert!(book.is_fully_fulfilled().unwrap());
        assert_eq!(book.journal().len(), 4);
    }

    #[test]
    fn partial_acceptance_releases_only_the_unaccepted_remainder() {
        let mut book = ledger(10);
        let first = claim("tranche-partial", 5, 2);
        book.record_claim(first.clone(), 3).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &first, "verify-partial", "auditor-1", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(3), 4
            )).unwrap(),
            TrancheState::PartiallyAccepted
        );
        assert_eq!(book.accepted_quantity(), Ok(3));
        let replacement = claim("tranche-replacement", 7, 5);
        book.record_claim(replacement.clone(), 6).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &replacement, "verify-replacement", "auditor-2", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(7), 7
            )).unwrap(),
            TrancheState::Accepted
        );
        assert_eq!(book.accepted_quantity(), Ok(10));
    }

    #[test]
    fn pending_claims_cannot_overcommit_the_requested_quantity() {
        let mut book = ledger(10);
        book.record_claim(claim("tranche-a", 6, 2), 3).unwrap();
        assert_eq!(
            book.record_claim(claim("tranche-b", 5, 4), 4),
            Err(TrancheError::OverCommitment)
        );
        assert_eq!(book.journal().len(), 1);
    }

    #[test]
    fn duplicate_claim_identity_is_rejected_without_mutation() {
        let mut book = ledger(10);
        book.record_claim(claim("tranche-a", 3, 2), 3).unwrap();
        assert_eq!(
            book.record_claim(claim("tranche-a", 3, 4), 4),
            Err(TrancheError::DuplicateClaim)
        );
        assert_eq!(book.journal().len(), 1);
    }

    #[test]
    fn duplicate_claim_evidence_is_rejected_without_mutation() {
        let mut book = ledger(10);
        let first = claim("tranche-a", 3, 2);
        book.record_claim(first.clone(), 3).unwrap();
        let mut replay = claim("tranche-b", 3, 4);
        replay.evidence_id = first.evidence_id.clone();
        assert_eq!(
            book.record_claim(replay, 4),
            Err(TrancheError::DuplicateEvidence)
        );
        assert_eq!(book.journal().len(), 1);
    }

    #[test]
    fn synthetic_origin_never_counts_as_verified_fulfilment() {
        let mut book = ledger(5);
        let mut synthetic = claim("tranche-synthetic", 5, 2);
        synthetic.origin = EvidenceOrigin::SyntheticSimulation;
        book.record_claim(synthetic.clone(), 3).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &synthetic, "verify-synthetic", "auditor-1", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(5), 4
            )).unwrap(),
            TrancheState::Indeterminate
        );
        assert_eq!(book.accepted_quantity(), Ok(0));
        assert!(!book.is_fully_fulfilled().unwrap());
    }

    #[test]
    fn unknown_verification_can_be_retried_with_new_live_evidence() {
        let mut book = ledger(5);
        let tranche = claim("tranche-retry", 5, 2);
        book.record_claim(tranche.clone(), 3).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "verify-unknown", "auditor-1", EvidenceStatus::Unknown,
                EvidenceOrigin::LiveDomain, Some(5), 4
            )).unwrap(),
            TrancheState::Indeterminate
        );
        assert_eq!(book.accepted_quantity(), Ok(0));
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "verify-live", "auditor-2", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(5), 5
            )).unwrap(),
            TrancheState::Accepted
        );
        assert_eq!(book.accepted_quantity(), Ok(5));
        assert_eq!(book.tranche("tranche-retry").unwrap().verification_history.len(), 2);
        assert_eq!(book.journal().len(), 3);
    }

    #[test]
    fn mismatched_claim_identity_is_rejected() {
        let mut book = ledger(5);
        let tranche = claim("tranche-bound", 5, 2);
        book.record_claim(tranche.clone(), 3).unwrap();
        let mut result = verification(
            &tranche, "verify-bound", "auditor-1", EvidenceStatus::Verified,
            EvidenceOrigin::LiveDomain, Some(5), 4
        );
        result.claim_evidence_id = "other-claim".to_owned();
        assert_eq!(
            book.verify_claim(result),
            Err(TrancheError::InvalidVerification)
        );
        assert_eq!(book.accepted_quantity(), Ok(0));
        assert_eq!(book.journal().len(), 1);
    }

    #[test]
    fn verifier_must_be_independent_of_requester_and_provider() {
        let mut book = ledger(5);
        let tranche = claim("tranche-independent", 5, 2);
        book.record_claim(tranche.clone(), 3).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "bad-verifier", "repair-coop", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(5), 4
            )),
            Err(TrancheError::NotIndependentVerifier)
        );
        assert_eq!(book.accepted_quantity(), Ok(0));
        assert_eq!(book.journal().len(), 1);
    }

    #[test]
    fn overfulfilment_decision_is_rejected_without_mutation() {
        let mut book = ledger(5);
        let tranche = claim("tranche-over", 3, 2);
        book.record_claim(tranche.clone(), 3).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "verify-over", "auditor-1", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(4), 4
            )),
            Err(TrancheError::OverFulfillment)
        );
        assert_eq!(book.accepted_quantity(), Ok(0));
        assert_eq!(book.tranche("tranche-over").unwrap().state, TrancheState::Claimed);
        assert_eq!(book.journal().len(), 1);
    }

    #[test]
    fn disputed_claim_requires_a_distinct_resolver_and_preserves_history() {
        let mut book = ledger(5);
        let tranche = claim("tranche-dispute", 5, 2);
        book.record_claim(tranche.clone(), 3).unwrap();
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "verify-zero", "auditor-1", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(0), 4
            )).unwrap(),
            TrancheState::Disputed
        );
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "verify-still-same", "auditor-1", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(5), 5
            )),
            Err(TrancheError::NotIndependentVerifier)
        );
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "verify-resolved", "auditor-2", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(5), 6
            )).unwrap(),
            TrancheState::Accepted
        );
        assert_eq!(book.accepted_quantity(), Ok(5));
        assert_eq!(book.tranche("tranche-dispute").unwrap().verification_history.len(), 2);
    }

    #[test]
    fn disputed_claim_keeps_quantity_reserved_until_resolved() {
        let mut book = ledger(5);
        let tranche = claim("tranche-reserved", 5, 2);
        book.record_claim(tranche.clone(), 3).unwrap();
        book.verify_claim(verification(
            &tranche, "verify-zero", "auditor-1", EvidenceStatus::Verified,
            EvidenceOrigin::LiveDomain, Some(0), 4
        )).unwrap();
        assert_eq!(
            book.record_claim(claim("tranche-too-early", 1, 5), 5),
            Err(TrancheError::OverCommitment)
        );
        assert_eq!(book.journal().len(), 2);
    }

    #[test]
    fn late_or_wrong_provider_claims_are_rejected() {
        let mut book = ledger(5);
        let mut late = claim("tranche-late", 1, 60);
        assert_eq!(book.record_claim(late.clone(), 61), Err(TrancheError::DeadlinePassed));
        late.observed_at = 2;
        late.source_id = "other-provider".to_owned();
        assert_eq!(book.record_claim(late, 3), Err(TrancheError::InvalidClaim));
        assert!(book.journal().is_empty());
    }

    #[test]
    fn out_of_order_journal_events_are_rejected() {
        let mut book = ledger(10);
        book.record_claim(claim("tranche-time", 5, 2), 4).unwrap();
        let tranche = book.tranche("tranche-time").unwrap().claim.clone();
        assert_eq!(
            book.verify_claim(verification(
                &tranche, "verify-time", "auditor-1", EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain, Some(5), 3
            )),
            Err(TrancheError::NonMonotonicJournalTime)
        );
        assert_eq!(book.journal().len(), 1);
    }
}

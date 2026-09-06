// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact rebind of persisted SSF registration evidence to the candidate lineage
//! that produced it.
//!
//! Direct in-memory registration and historical restart reconciliation converge
//! here before any authority-issuance layer. A persisted record or a rehydrated
//! candidate is insufficient alone; the exact registration subject, record,
//! verifier evidence, and candidate lineage must agree.
//!
//! This crate remains non-authoritative.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_adoption_authority_candidate::AdoptionAuthorityCandidateV1;
use mycelix_ssf_adoption_authority_registration::{
    AdoptionAuthorityRegistrationReceiptV1, AdoptionAuthorityRegistrationStoreV1,
    AdoptionAuthorityRegistrationSubjectV1, DurableRegistrationFrontierV1,
    PersistedAdoptionAuthorityRegistrationV1, RegistrationAttemptManifestV1,
    RegistrationDispositionV1, RegistrationReceiptCommitment, RegistrationRecordCommitment,
    RegistrationStoreDescriptorV1,
};
use mycelix_ssf_historical_registration_reconciliation::{
    HistoricalReconciliationDispositionV1, HistoricalRegistrationReconciliationV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PersistedRegistrationEvidenceKindV1 {
    DirectRegistration,
    HistoricalReconciliation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistedRegistrationBindingV1 {
    pub kind: PersistedRegistrationEvidenceKindV1,
    pub manifest: RegistrationAttemptManifestV1,
    pub record: RegistrationRecordCommitment,
    pub post_frontier: DurableRegistrationFrontierV1,
    pub verification_descriptor: RegistrationStoreDescriptorV1,
    pub evidence_receipt: RegistrationReceiptCommitment,
    pub evidence_valid_until: u64,
    pub authority_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistedRegistrationRebindError {
    RegistrationSubjectMismatch,
    SourceDispositionNotPersisted,
    HistoricalReceiptMissing,
    HistoricalAuthorityCeilingMissing,
}

fn subject_from_candidate<P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    candidate: &AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
) -> AdoptionAuthorityRegistrationSubjectV1 {
    AdoptionAuthorityRegistrationSubjectV1 {
        candidate_id: candidate.candidate_id(),
        nonce: candidate.nonce(),
        operation: candidate.operation(),
        source_local_state: candidate.source_local_state(),
        remote_target: candidate.remote_target(),
        policy_receipt: candidate.policy_receipt_commitment(),
        scope: candidate.scope(),
        consequence: candidate.consequence(),
        candidate_valid_until: candidate.valid_until(),
    }
}

fn exact_subject_matches<P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    candidate: &AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
    manifest: RegistrationAttemptManifestV1,
) -> bool {
    subject_from_candidate(candidate) == manifest.subject()
}

enum PersistedCandidateCarrierV1<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    Direct(PersistedAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>),
    Historical {
        candidate: AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
        reconciliation: HistoricalRegistrationReconciliationV1<S>,
    },
}

/// Exact persisted registration rebound to the full typed candidate lineage.
///
/// This type intentionally owns the source evidence rather than copying only
/// endpoint fields. Later authority issuance can therefore retain the full
/// provenance chain.
pub struct ReboundPersistedAdoptionRegistrationV1<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    carrier: PersistedCandidateCarrierV1<S, P, EV, RV, AQ, LQ, L, R>,
    binding: PersistedRegistrationBindingV1,
}

impl<S, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    ReboundPersistedAdoptionRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn binding(&self) -> PersistedRegistrationBindingV1 {
        self.binding
    }

    pub fn candidate(&self) -> &AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R> {
        match &self.carrier {
            PersistedCandidateCarrierV1::Direct(registration) => registration.candidate(),
            PersistedCandidateCarrierV1::Historical { candidate, .. } => candidate,
        }
    }

    pub const fn authority_eligibility_valid_until(&self) -> u64 {
        self.binding.authority_eligibility_valid_until
    }

    pub const fn evidence_valid_until(&self) -> u64 {
        self.binding.evidence_valid_until
    }

    /// Historical source evidence remains retained by value for audit and
    /// later issuance rebind. Direct registrations return `None`.
    pub fn historical_reconciliation(
        &self,
    ) -> Option<&HistoricalRegistrationReconciliationV1<S>> {
        match &self.carrier {
            PersistedCandidateCarrierV1::Direct(_) => None,
            PersistedCandidateCarrierV1::Historical { reconciliation, .. } => {
                Some(reconciliation)
            }
        }
    }

    /// Persisted registration has not yet been consumed for issuance. A later
    /// durable single-use issuance reservation is required to prevent one
    /// registration record from minting multiple authority leases.
    pub const fn requires_single_use_issuance_reservation(&self) -> bool {
        true
    }

    pub const fn contains_current_authority_revalidation(&self) -> bool {
        false
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

fn persisted_fields_from_receipt(
    receipt: &AdoptionAuthorityRegistrationReceiptV1,
) -> Result<
    (RegistrationRecordCommitment, DurableRegistrationFrontierV1),
    PersistedRegistrationRebindError,
> {
    match receipt.disposition {
        RegistrationDispositionV1::Persisted {
            record,
            post_frontier,
        } => Ok((record, post_frontier)),
        _ => Err(PersistedRegistrationRebindError::SourceDispositionNotPersisted),
    }
}

/// Rebind the direct in-memory persisted path.
///
/// The parent persisted typestate already owns the candidate. This function
/// rechecks the exact registration subject defensively and retains the source
/// token by value.
pub fn rebind_direct_persisted_registration<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    registration: PersistedAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>,
) -> Result<
    ReboundPersistedAdoptionRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>,
    PersistedRegistrationRebindError,
>
where
    S: AdoptionAuthorityRegistrationStoreV1,
{
    let manifest = registration.manifest();
    if !exact_subject_matches(registration.candidate(), manifest) {
        return Err(PersistedRegistrationRebindError::RegistrationSubjectMismatch);
    }

    let receipt = registration.receipt();
    let (record, post_frontier) = persisted_fields_from_receipt(receipt)?;
    let binding = PersistedRegistrationBindingV1 {
        kind: PersistedRegistrationEvidenceKindV1::DirectRegistration,
        manifest,
        record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        authority_eligibility_valid_until: registration
            .valid_until()
            .min(registration.candidate().valid_until()),
    };

    Ok(ReboundPersistedAdoptionRegistrationV1 {
        carrier: PersistedCandidateCarrierV1::Direct(registration),
        binding,
    })
}

/// Rebind restart-recovered historical persistence to an independently
/// rehydrated exact candidate lineage.
///
/// A historical persisted fact alone cannot issue authority, and a rehydrated
/// candidate alone cannot claim it was registered. Both must agree on the exact
/// registration subject.
pub fn rebind_historical_persisted_registration<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    reconciliation: HistoricalRegistrationReconciliationV1<S>,
    candidate: AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
) -> Result<
    ReboundPersistedAdoptionRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>,
    PersistedRegistrationRebindError,
>
where
    S: AdoptionAuthorityRegistrationStoreV1,
{
    let manifest = reconciliation.manifest();
    if !exact_subject_matches(&candidate, manifest) {
        return Err(PersistedRegistrationRebindError::RegistrationSubjectMismatch);
    }

    let (record, post_frontier) = match reconciliation.disposition() {
        HistoricalReconciliationDispositionV1::Persisted {
            record,
            post_frontier,
        } => (record, post_frontier),
        _ => return Err(PersistedRegistrationRebindError::SourceDispositionNotPersisted),
    };

    let receipt = reconciliation
        .receipt()
        .ok_or(PersistedRegistrationRebindError::HistoricalReceiptMissing)?;
    let authority_eligibility_valid_until = reconciliation
        .authority_eligibility_valid_until()
        .ok_or(PersistedRegistrationRebindError::HistoricalAuthorityCeilingMissing)?
        .min(candidate.valid_until());

    let binding = PersistedRegistrationBindingV1 {
        kind: PersistedRegistrationEvidenceKindV1::HistoricalReconciliation,
        manifest,
        record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        authority_eligibility_valid_until,
    };

    Ok(ReboundPersistedAdoptionRegistrationV1 {
        carrier: PersistedCandidateCarrierV1::Historical {
            candidate,
            reconciliation,
        },
        binding,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn min_authority_ceiling(evidence: u64, candidate: u64) -> u64 {
        evidence.min(candidate)
    }

    #[test]
    fn historical_evidence_can_be_fresher_without_refreshing_authority() {
        assert_eq!(min_authority_ceiling(180, 90), 90);
    }

    #[test]
    fn evidence_kind_keeps_direct_and_historical_paths_distinct() {
        assert_ne!(
            PersistedRegistrationEvidenceKindV1::DirectRegistration,
            PersistedRegistrationEvidenceKindV1::HistoricalReconciliation
        );
    }
}

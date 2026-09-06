// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact rebind of durable authority-lease issuance evidence to the bounded
//! lease-candidate lineage that produced it.
//!
//! Direct in-memory issuance and historical restart reconciliation converge
//! here before any use-time effect-admission layer. Neither a historical lease
//! record nor a rehydrated candidate is sufficient alone.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_authority_lease_issuance::{
    AuthorityLeaseIssuanceDispositionV1, AuthorityLeaseIssuanceReceiptCommitment,
    AuthorityLeaseIssuanceAttemptManifestV1, AuthorityLeaseIssuanceStoreV1,
    AuthorityLeaseRecordCommitment, AuthorityLeaseStoreDescriptorV1,
    DurableAuthorityLeaseFrontierV1, DurablyIssuedAuthorityLeaseV1,
};
use mycelix_ssf_bounded_authority_lease_candidate::BoundedAuthorityLeaseCandidateV1;
use mycelix_ssf_historical_authority_lease_reconciliation::{
    HistoricalAuthorityLeaseReconciliationDispositionV1,
    HistoricalAuthorityLeaseReconciliationV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IssuedAuthorityLeaseEvidenceKindV1 {
    DirectIssuance,
    HistoricalReconciliation,
}

/// Exact durable lease issuance evidence rebound to one typed bounded-candidate
/// lineage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IssuedAuthorityLeaseBindingV1 {
    pub kind: IssuedAuthorityLeaseEvidenceKindV1,
    pub manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    pub lease_record: AuthorityLeaseRecordCommitment,
    pub post_frontier: DurableAuthorityLeaseFrontierV1,
    pub verification_descriptor: AuthorityLeaseStoreDescriptorV1,
    pub evidence_receipt: AuthorityLeaseIssuanceReceiptCommitment,
    pub evidence_valid_until: u64,
    pub authority_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssuedAuthorityLeaseRebindError {
    IssuanceSubjectMismatch,
    SourceDispositionNotIssued,
    HistoricalReceiptMissing,
    HistoricalAuthorityCeilingMissing,
}

fn subject_matches_candidate<
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    CQ,
    BQ,
    TQ,
    const L: usize,
    const R: usize,
>(
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    candidate: &BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
) -> bool {
    let subject = manifest.subject();
    subject.reserved_issuance == candidate.revalidated().reserved().binding()
        && subject.lease_candidate_id == candidate.lease_candidate_id()
        && subject.nonce == candidate.nonce()
        && subject.operation == candidate.operation()
        && subject.source_local_state == candidate.source_local_state()
        && subject.remote_target == candidate.remote_target()
        && subject.scope == candidate.scope()
        && subject.consequence == candidate.consequence()
        && subject.lease_valid_until == candidate.valid_until()
}

enum IssuedAuthorityLeaseCarrierV1<
    S,
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    CQ,
    BQ,
    TQ,
    const L: usize,
    const R: usize,
> {
    Direct(
        DurablyIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    ),
    Historical {
        candidate:
            BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
        reconciliation: HistoricalAuthorityLeaseReconciliationV1<S>,
    },
}

/// Recovered exact semantic lease lineage.
///
/// The historical path can reconstruct this only after exact issuance-subject
/// equality with an independently rehydrated bounded candidate. The token is
/// still not an actuator capability and may be stale at use time.
pub struct ReboundIssuedAuthorityLeaseV1<
    S,
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    CQ,
    BQ,
    TQ,
    const L: usize,
    const R: usize,
> {
    carrier: IssuedAuthorityLeaseCarrierV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    binding: IssuedAuthorityLeaseBindingV1,
}

impl<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    ReboundIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn binding(&self) -> IssuedAuthorityLeaseBindingV1 {
        self.binding
    }

    pub fn candidate(
        &self,
    ) -> &BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
        match &self.carrier {
            IssuedAuthorityLeaseCarrierV1::Direct(issued) => issued.candidate(),
            IssuedAuthorityLeaseCarrierV1::Historical { candidate, .. } => candidate,
        }
    }

    pub fn historical_reconciliation(
        &self,
    ) -> Option<&HistoricalAuthorityLeaseReconciliationV1<S>> {
        match &self.carrier {
            IssuedAuthorityLeaseCarrierV1::Direct(_) => None,
            IssuedAuthorityLeaseCarrierV1::Historical { reconciliation, .. } => {
                Some(reconciliation)
            }
        }
    }

    pub const fn authority_valid_until(&self) -> u64 {
        self.binding.authority_valid_until
    }

    pub const fn semantic_lease_lineage_established(&self) -> bool {
        true
    }

    pub const fn requires_use_time_revalidation(&self) -> bool {
        true
    }

    pub const fn eligible_for_effect_admission_without_revalidation(&self) -> bool {
        false
    }

    pub const fn delegation_allowed(&self) -> bool {
        false
    }

    pub const fn contains_direct_state_install_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

fn issued_fields(
    disposition: AuthorityLeaseIssuanceDispositionV1,
) -> Result<
    (AuthorityLeaseRecordCommitment, DurableAuthorityLeaseFrontierV1),
    IssuedAuthorityLeaseRebindError,
> {
    match disposition {
        AuthorityLeaseIssuanceDispositionV1::Issued {
            lease_record,
            post_frontier,
        } => Ok((lease_record, post_frontier)),
        _ => Err(IssuedAuthorityLeaseRebindError::SourceDispositionNotIssued),
    }
}

/// Rebind the direct in-memory issuance path defensively to the exact candidate
/// subject retained by the durable manifest.
pub fn rebind_direct_issued_authority_lease<
    S,
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    CQ,
    BQ,
    TQ,
    const L: usize,
    const R: usize,
>(
    issued: DurablyIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
) -> Result<
    ReboundIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    IssuedAuthorityLeaseRebindError,
>
where
    S: AuthorityLeaseIssuanceStoreV1,
{
    let manifest = issued.manifest();
    if !subject_matches_candidate(manifest, issued.candidate()) {
        return Err(IssuedAuthorityLeaseRebindError::IssuanceSubjectMismatch);
    }

    let receipt = issued.receipt();
    let (lease_record, post_frontier) = issued_fields(receipt.disposition)?;
    let binding = IssuedAuthorityLeaseBindingV1 {
        kind: IssuedAuthorityLeaseEvidenceKindV1::DirectIssuance,
        manifest,
        lease_record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        authority_valid_until: issued.authority_valid_until(),
    };

    Ok(ReboundIssuedAuthorityLeaseV1 {
        carrier: IssuedAuthorityLeaseCarrierV1::Direct(issued),
        binding,
    })
}

/// Rebind historical `Issued` evidence to an independently rehydrated exact
/// bounded candidate lineage.
pub fn rebind_historical_issued_authority_lease<
    S,
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    CQ,
    BQ,
    TQ,
    const L: usize,
    const R: usize,
>(
    reconciliation: HistoricalAuthorityLeaseReconciliationV1<S>,
    candidate: BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
) -> Result<
    ReboundIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    IssuedAuthorityLeaseRebindError,
>
where
    S: AuthorityLeaseIssuanceStoreV1,
{
    let manifest = reconciliation.manifest();
    if !subject_matches_candidate(manifest, &candidate) {
        return Err(IssuedAuthorityLeaseRebindError::IssuanceSubjectMismatch);
    }

    let (lease_record, post_frontier) = match reconciliation.disposition() {
        HistoricalAuthorityLeaseReconciliationDispositionV1::Issued {
            lease_record,
            post_frontier,
        } => (lease_record, post_frontier),
        _ => return Err(IssuedAuthorityLeaseRebindError::SourceDispositionNotIssued),
    };

    let receipt = reconciliation
        .receipt()
        .ok_or(IssuedAuthorityLeaseRebindError::HistoricalReceiptMissing)?;
    let authority_valid_until = reconciliation
        .authority_eligibility_valid_until()
        .ok_or(IssuedAuthorityLeaseRebindError::HistoricalAuthorityCeilingMissing)?
        .min(candidate.valid_until());

    let binding = IssuedAuthorityLeaseBindingV1 {
        kind: IssuedAuthorityLeaseEvidenceKindV1::HistoricalReconciliation,
        manifest,
        lease_record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        authority_valid_until,
    };

    Ok(ReboundIssuedAuthorityLeaseV1 {
        carrier: IssuedAuthorityLeaseCarrierV1::Historical {
            candidate,
            reconciliation,
        },
        binding,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_and_historical_evidence_kinds_remain_distinct() {
        assert_ne!(
            IssuedAuthorityLeaseEvidenceKindV1::DirectIssuance,
            IssuedAuthorityLeaseEvidenceKindV1::HistoricalReconciliation
        );
    }

    #[test]
    fn fresh_historical_fact_never_refreshes_old_lease() {
        let historical_ceiling = 180;
        let candidate_ceiling = 90;
        assert_eq!(historical_ceiling.min(candidate_ceiling), 90);
    }
}

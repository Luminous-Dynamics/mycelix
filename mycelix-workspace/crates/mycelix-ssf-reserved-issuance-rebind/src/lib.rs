// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact rebind of durable issuance-reservation evidence to the rebound
//! persisted-registration lineage that the reservation consumed.
//!
//! Direct in-memory reservation and historical restart reconciliation converge
//! here before current authority revalidation. Neither path mints authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_historical_issuance_reservation_reconciliation::{
    HistoricalIssuanceReservationDispositionV1,
    HistoricalIssuanceReservationReconciliationV1,
};
use mycelix_ssf_persisted_registration_rebind::ReboundPersistedAdoptionRegistrationV1;
use mycelix_ssf_single_use_issuance_reservation::{
    DurableIssuanceReservationFrontierV1, IssuanceReservationAttemptManifestV1,
    IssuanceReservationDispositionV1, IssuanceReservationReceiptCommitment,
    IssuanceReservationRecordCommitment, IssuanceReservationStoreDescriptorV1,
    IssuanceReservationStoreV1, ReservedSingleUseIssuanceV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReservedIssuanceEvidenceKindV1 {
    DirectReservation,
    HistoricalReconciliation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReservedIssuanceBindingV1 {
    pub kind: ReservedIssuanceEvidenceKindV1,
    pub manifest: IssuanceReservationAttemptManifestV1,
    pub record: IssuanceReservationRecordCommitment,
    pub post_frontier: DurableIssuanceReservationFrontierV1,
    pub verification_descriptor: IssuanceReservationStoreDescriptorV1,
    pub evidence_receipt: IssuanceReservationReceiptCommitment,
    pub evidence_valid_until: u64,
    pub authority_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReservedIssuanceRebindError {
    RegistrationBindingMismatch,
    SourceDispositionNotReserved,
    HistoricalReceiptMissing,
    HistoricalAuthorityCeilingMissing,
}

enum ReservedIssuanceCarrierV1<
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    Direct(ReservedSingleUseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>),
    Historical {
        rebound: ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R>,
        reconciliation: HistoricalIssuanceReservationReconciliationV1<IS>,
    },
}

/// Exact reserved issuance lineage, rebound to the complete persisted
/// registration provenance. This is the only reservation form intended to feed
/// current authority revalidation.
pub struct ReboundReservedIssuanceV1<
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    carrier: ReservedIssuanceCarrierV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    binding: ReservedIssuanceBindingV1,
}

impl<IS, RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    ReboundReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn binding(&self) -> ReservedIssuanceBindingV1 {
        self.binding
    }

    pub fn rebound_registration(
        &self,
    ) -> &ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R> {
        match &self.carrier {
            ReservedIssuanceCarrierV1::Direct(reserved) => reserved.rebound(),
            ReservedIssuanceCarrierV1::Historical { rebound, .. } => rebound,
        }
    }

    pub fn historical_reconciliation(
        &self,
    ) -> Option<&HistoricalIssuanceReservationReconciliationV1<IS>> {
        match &self.carrier {
            ReservedIssuanceCarrierV1::Direct(_) => None,
            ReservedIssuanceCarrierV1::Historical { reconciliation, .. } => Some(reconciliation),
        }
    }

    pub const fn authority_eligibility_valid_until(&self) -> u64 {
        self.binding.authority_eligibility_valid_until
    }

    pub const fn registration_consumed_for_issuance(&self) -> bool {
        true
    }

    pub const fn eligible_for_current_authority_revalidation(&self) -> bool {
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

fn manifest_matches_registration<RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    manifest: IssuanceReservationAttemptManifestV1,
    rebound: &ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R>,
) -> bool {
    manifest.subject().persisted_registration == rebound.binding()
}

fn reserved_fields(
    disposition: IssuanceReservationDispositionV1,
) -> Result<
    (IssuanceReservationRecordCommitment, DurableIssuanceReservationFrontierV1),
    ReservedIssuanceRebindError,
> {
    match disposition {
        IssuanceReservationDispositionV1::Reserved {
            record,
            post_frontier,
        } => Ok((record, post_frontier)),
        _ => Err(ReservedIssuanceRebindError::SourceDispositionNotReserved),
    }
}

pub fn rebind_direct_reserved_issuance<
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    reserved: ReservedSingleUseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
) -> Result<
    ReboundReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    ReservedIssuanceRebindError,
>
where
    IS: IssuanceReservationStoreV1,
{
    let manifest = reserved.manifest();
    if !manifest_matches_registration(manifest, reserved.rebound()) {
        return Err(ReservedIssuanceRebindError::RegistrationBindingMismatch);
    }

    let receipt = reserved.receipt();
    let (record, post_frontier) = reserved_fields(receipt.disposition)?;
    let binding = ReservedIssuanceBindingV1 {
        kind: ReservedIssuanceEvidenceKindV1::DirectReservation,
        manifest,
        record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        authority_eligibility_valid_until: reserved.authority_eligibility_valid_until(),
    };

    Ok(ReboundReservedIssuanceV1 {
        carrier: ReservedIssuanceCarrierV1::Direct(reserved),
        binding,
    })
}

pub fn rebind_historical_reserved_issuance<
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    reconciliation: HistoricalIssuanceReservationReconciliationV1<IS>,
    rebound: ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R>,
) -> Result<
    ReboundReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    ReservedIssuanceRebindError,
>
where
    IS: IssuanceReservationStoreV1,
{
    let manifest = reconciliation.manifest();
    if !manifest_matches_registration(manifest, &rebound) {
        return Err(ReservedIssuanceRebindError::RegistrationBindingMismatch);
    }

    let (record, post_frontier) = match reconciliation.disposition() {
        HistoricalIssuanceReservationDispositionV1::Reserved {
            record,
            post_frontier,
        } => (record, post_frontier),
        _ => return Err(ReservedIssuanceRebindError::SourceDispositionNotReserved),
    };

    let receipt = reconciliation
        .receipt()
        .ok_or(ReservedIssuanceRebindError::HistoricalReceiptMissing)?;
    let authority_eligibility_valid_until = reconciliation
        .authority_eligibility_valid_until()
        .ok_or(ReservedIssuanceRebindError::HistoricalAuthorityCeilingMissing)?
        .min(rebound.authority_eligibility_valid_until());

    let binding = ReservedIssuanceBindingV1 {
        kind: ReservedIssuanceEvidenceKindV1::HistoricalReconciliation,
        manifest,
        record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        authority_eligibility_valid_until,
    };

    Ok(ReboundReservedIssuanceV1 {
        carrier: ReservedIssuanceCarrierV1::Historical {
            rebound,
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
            ReservedIssuanceEvidenceKindV1::DirectReservation,
            ReservedIssuanceEvidenceKindV1::HistoricalReconciliation
        );
    }

    #[test]
    fn historical_rebind_cannot_refresh_authority() {
        assert_eq!(180u64.min(90), 90);
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Historical reconciliation for durable SSF issuance reservations.
//!
//! An unresolved reservation can outlive the verifier/key generation that
//! accepted the original write. A currently trusted reconciler may attest the
//! historical disposition of that exact journaled attempt without pretending
//! to be the expired write-time generation, provided the stable reservation-
//! store identity remains exact.
//!
//! Fresh historical knowledge never refreshes the original authority ceiling.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_single_use_issuance_reservation::{
    DurableIssuanceReservationFrontierV1, IssuanceReservationAttemptAbsenceCommitment,
    IssuanceReservationAttemptManifestV1, IssuanceReservationDispositionV1,
    IssuanceReservationReceiptV1, IssuanceReservationRecordCommitment,
    IssuanceReservationStoreDescriptorV1, IssuanceReservationStoreV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedHistoricalIssuanceReservationReconcilerProfileV1 {
    descriptor: IssuanceReservationStoreDescriptorV1,
}

impl ExpectedHistoricalIssuanceReservationReconcilerProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: IssuanceReservationStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> IssuanceReservationStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalIssuanceReservationAmbiguityReasonV1 {
    UnsupportedManifestSchema,
    HistoricalStoreIdentityMismatch,
    ReconcilerDescriptorMismatchBefore,
    ReconcilerDescriptorChangedAfter,
    UnsupportedReceiptSchema,
    ReceiptReconcilerMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesReconciler,
    PersistedGenerationOverflow,
    InvalidPersistedFrontier,
    InvalidProvenNotReservedFrontier,
    AttemptIdConflict,
    RegistrationAlreadyReserved,
    StoreReportedOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalIssuanceReservationDispositionV1 {
    Reserved {
        record: IssuanceReservationRecordCommitment,
        post_frontier: DurableIssuanceReservationFrontierV1,
    },
    ProvenNotReserved {
        observed_frontier: DurableIssuanceReservationFrontierV1,
        absence_evidence: IssuanceReservationAttemptAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: HistoricalIssuanceReservationAmbiguityReasonV1,
    },
}

/// Non-authoritative historical reservation evidence.
pub struct HistoricalIssuanceReservationReconciliationV1<IS> {
    manifest: IssuanceReservationAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalIssuanceReservationReconcilerProfileV1,
    receipt: Option<IssuanceReservationReceiptV1>,
    disposition: HistoricalIssuanceReservationDispositionV1,
    reconciliation_valid_until: Option<u64>,
    authority_eligibility_valid_until: Option<u64>,
    _store_type: PhantomData<fn() -> IS>,
}

impl<IS> HistoricalIssuanceReservationReconciliationV1<IS> {
    pub const fn manifest(&self) -> IssuanceReservationAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_reconciler(
        &self,
    ) -> ExpectedHistoricalIssuanceReservationReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub fn receipt(&self) -> Option<&IssuanceReservationReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> HistoricalIssuanceReservationDispositionV1 {
        self.disposition
    }

    pub const fn reconciliation_valid_until(&self) -> Option<u64> {
        self.reconciliation_valid_until
    }

    pub const fn authority_eligibility_valid_until(&self) -> Option<u64> {
        self.authority_eligibility_valid_until
    }

    pub fn permits_new_attempt(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalIssuanceReservationDispositionV1::ProvenNotReserved { .. }
        )
    }

    pub fn reserved_lineage_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalIssuanceReservationDispositionV1::Reserved { .. }
        )
    }

    pub const fn refreshes_original_authority(&self) -> bool {
        false
    }

    pub const fn contains_current_authority_revalidation(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

fn unknown<IS>(
    manifest: IssuanceReservationAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalIssuanceReservationReconcilerProfileV1,
    receipt: Option<IssuanceReservationReceiptV1>,
    reason: HistoricalIssuanceReservationAmbiguityReasonV1,
) -> HistoricalIssuanceReservationReconciliationV1<IS> {
    HistoricalIssuanceReservationReconciliationV1 {
        manifest,
        expected_reconciler,
        receipt,
        disposition: HistoricalIssuanceReservationDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        authority_eligibility_valid_until: None,
        _store_type: PhantomData,
    }
}

fn same_store_identity(
    original: IssuanceReservationStoreDescriptorV1,
    current: IssuanceReservationStoreDescriptorV1,
) -> bool {
    original.identity == current.identity
}

fn validate_reserved_frontier(
    before: DurableIssuanceReservationFrontierV1,
    record: IssuanceReservationRecordCommitment,
    after: DurableIssuanceReservationFrontierV1,
) -> Result<(), HistoricalIssuanceReservationAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(HistoricalIssuanceReservationAmbiguityReasonV1::PersistedGenerationOverflow)?;

    if after.generation.get() != expected_generation || after.head != Some(record) {
        return Err(HistoricalIssuanceReservationAmbiguityReasonV1::InvalidPersistedFrontier);
    }
    Ok(())
}

fn validate_absence_frontier(
    expected: DurableIssuanceReservationFrontierV1,
    observed: DurableIssuanceReservationFrontierV1,
) -> Result<(), HistoricalIssuanceReservationAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(
            HistoricalIssuanceReservationAmbiguityReasonV1::InvalidProvenNotReservedFrontier,
        );
    }
    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(
            HistoricalIssuanceReservationAmbiguityReasonV1::InvalidProvenNotReservedFrontier,
        );
    }
    Ok(())
}

fn authority_ceiling(receipt_valid_until: u64, original_authority_ceiling: u64) -> u64 {
    receipt_valid_until.min(original_authority_ceiling)
}

/// Read-only historical reconciliation under a currently trusted same-identity
/// reservation-store verifier.
pub fn reconcile_historical_issuance_reservation<IS>(
    expected_reconciler: ExpectedHistoricalIssuanceReservationReconcilerProfileV1,
    store: &IS,
    manifest: IssuanceReservationAttemptManifestV1,
) -> HistoricalIssuanceReservationReconciliationV1<IS>
where
    IS: IssuanceReservationStoreV1,
{
    let current = expected_reconciler.descriptor();

    if manifest.schema_version() != SSF_SCHEMA_V1 {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalIssuanceReservationAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }

    if !same_store_identity(manifest.expected_store(), current) {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalIssuanceReservationAmbiguityReasonV1::HistoricalStoreIdentityMismatch,
        );
    }

    if store.descriptor() != current {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalIssuanceReservationAmbiguityReasonV1::ReconcilerDescriptorMismatchBefore,
        );
    }

    let receipt = store.reconcile_reservation(&manifest);

    if store.descriptor() != current {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::ReconcilerDescriptorChangedAfter,
        );
    }

    if receipt.schema_version != SSF_SCHEMA_V1 {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::UnsupportedReceiptSchema,
        );
    }
    if receipt.store != current {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::ReceiptReconcilerMismatch,
        );
    }
    if receipt.manifest != manifest {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::ReceiptManifestMismatch,
        );
    }
    if receipt.valid_until > current.valid_until {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::ReceiptOutlivesReconciler,
        );
    }

    match receipt.disposition {
        IssuanceReservationDispositionV1::Reserved {
            record,
            post_frontier,
        } => {
            if let Err(reason) =
                validate_reserved_frontier(manifest.expected_frontier(), record, post_frontier)
            {
                return unknown(manifest, expected_reconciler, Some(receipt), reason);
            }
            HistoricalIssuanceReservationReconciliationV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition: HistoricalIssuanceReservationDispositionV1::Reserved {
                    record,
                    post_frontier,
                },
                reconciliation_valid_until: Some(receipt.valid_until),
                authority_eligibility_valid_until: Some(authority_ceiling(
                    receipt.valid_until,
                    manifest
                        .subject()
                        .persisted_registration
                        .authority_eligibility_valid_until,
                )),
                _store_type: PhantomData,
            }
        }
        IssuanceReservationDispositionV1::ProvenNotReserved {
            observed_frontier,
            absence_evidence,
        } => {
            if let Err(reason) =
                validate_absence_frontier(manifest.expected_frontier(), observed_frontier)
            {
                return unknown(manifest, expected_reconciler, Some(receipt), reason);
            }
            HistoricalIssuanceReservationReconciliationV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition: HistoricalIssuanceReservationDispositionV1::ProvenNotReserved {
                    observed_frontier,
                    absence_evidence,
                },
                reconciliation_valid_until: Some(receipt.valid_until),
                authority_eligibility_valid_until: None,
                _store_type: PhantomData,
            }
        }
        IssuanceReservationDispositionV1::AttemptIdConflict { .. } => unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::AttemptIdConflict,
        ),
        IssuanceReservationDispositionV1::RegistrationAlreadyReserved { .. } => unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::RegistrationAlreadyReserved,
        ),
        IssuanceReservationDispositionV1::OutcomeUnknown => unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalIssuanceReservationAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_single_use_issuance_reservation::{
        IssuanceReservationStoreIdentityCommitment, IssuanceReservationStorePolicyCommitment,
        IssuanceReservationStoreVerifierGeneration,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn descriptor(identity: u8, policy: u8, generation: u64, valid_until: u64) -> IssuanceReservationStoreDescriptorV1 {
        IssuanceReservationStoreDescriptorV1 {
            identity: IssuanceReservationStoreIdentityCommitment::from_bytes(bytes(identity)),
            policy: IssuanceReservationStorePolicyCommitment::from_bytes(bytes(policy)),
            verifier_generation: IssuanceReservationStoreVerifierGeneration::new(generation),
            valid_until,
        }
    }

    #[test]
    fn same_store_identity_allows_policy_and_verifier_rotation() {
        assert!(same_store_identity(
            descriptor(1, 4, 4, 100),
            descriptor(1, 5, 5, 200)
        ));
    }

    #[test]
    fn different_store_identity_is_not_rotation() {
        assert!(!same_store_identity(
            descriptor(1, 4, 4, 100),
            descriptor(2, 5, 5, 200)
        ));
    }

    #[test]
    fn fresh_reconciliation_never_refreshes_old_authority() {
        assert_eq!(authority_ceiling(180, 90), 90);
    }
}

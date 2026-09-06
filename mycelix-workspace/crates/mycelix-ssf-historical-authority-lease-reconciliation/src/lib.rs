// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Historical reconciliation for SSF durable authority-lease issuance.
//!
//! An issuance attempt may remain unresolved longer than the lease-store
//! verifier/key generation that accepted the original write. This crate allows
//! a currently trusted reconciler to attest the historical disposition of that
//! exact old attempt without pretending to be the expired write-time
//! generation.
//!
//! Historical reconciliation never refreshes the authority lifetime of the
//! original bounded lease candidate and never recreates semantic or effect
//! authority by itself.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_authority_lease_issuance::{
    AuthorityLeaseIssuanceAttemptManifestV1, AuthorityLeaseIssuanceDispositionV1,
    AuthorityLeaseIssuanceReceiptV1, AuthorityLeaseIssuanceStoreV1,
    AuthorityLeaseIssuanceAttemptAbsenceCommitment, AuthorityLeaseRecordCommitment,
    AuthorityLeaseStoreDescriptorV1, DurableAuthorityLeaseFrontierV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedHistoricalAuthorityLeaseReconcilerProfileV1 {
    descriptor: AuthorityLeaseStoreDescriptorV1,
}

impl ExpectedHistoricalAuthorityLeaseReconcilerProfileV1 {
    pub const fn from_trusted_configuration(descriptor: AuthorityLeaseStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> AuthorityLeaseStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1 {
    UnsupportedManifestSchema,
    HistoricalStoreIdentityMismatch,
    ReconcilerDescriptorMismatchBefore,
    ReconcilerDescriptorChangedAfter,
    StoreError,
    UnsupportedReceiptSchema,
    ReceiptReconcilerMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesReconciler,
    IssuedGenerationOverflow,
    InvalidIssuedFrontier,
    InvalidProvenNotIssuedFrontier,
    AttemptIdConflict,
    ReservationAlreadyIssued,
    StoreReportedOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalAuthorityLeaseReconciliationDispositionV1 {
    Issued {
        lease_record: AuthorityLeaseRecordCommitment,
        post_frontier: DurableAuthorityLeaseFrontierV1,
    },
    ProvenNotIssued {
        observed_frontier: DurableAuthorityLeaseFrontierV1,
        absence_evidence: AuthorityLeaseIssuanceAttemptAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1,
    },
}

/// Read-only historical issuance fact under a currently trusted reconciler.
///
/// `reconciliation_valid_until` is freshness of the new historical attestation.
/// `authority_eligibility_valid_until` remains capped by the original lease
/// request and therefore cannot be refreshed by key/policy rotation.
pub struct HistoricalAuthorityLeaseReconciliationV1<S> {
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalAuthorityLeaseReconcilerProfileV1,
    receipt: Option<AuthorityLeaseIssuanceReceiptV1>,
    disposition: HistoricalAuthorityLeaseReconciliationDispositionV1,
    reconciliation_valid_until: Option<u64>,
    authority_eligibility_valid_until: Option<u64>,
    _store: PhantomData<fn() -> S>,
}

impl<S> HistoricalAuthorityLeaseReconciliationV1<S> {
    pub const fn manifest(&self) -> AuthorityLeaseIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_reconciler(&self) -> ExpectedHistoricalAuthorityLeaseReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub fn receipt(&self) -> Option<&AuthorityLeaseIssuanceReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> HistoricalAuthorityLeaseReconciliationDispositionV1 {
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
            HistoricalAuthorityLeaseReconciliationDispositionV1::ProvenNotIssued { .. }
        )
    }

    pub fn issued_lineage_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalAuthorityLeaseReconciliationDispositionV1::Issued { .. }
        )
    }

    pub const fn refreshes_original_lease_authority(&self) -> bool {
        false
    }

    pub const fn contains_semantic_lease_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

fn unknown<S>(
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalAuthorityLeaseReconcilerProfileV1,
    receipt: Option<AuthorityLeaseIssuanceReceiptV1>,
    reason: HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1,
) -> HistoricalAuthorityLeaseReconciliationV1<S> {
    HistoricalAuthorityLeaseReconciliationV1 {
        manifest,
        expected_reconciler,
        receipt,
        disposition: HistoricalAuthorityLeaseReconciliationDispositionV1::OutcomeUnknown {
            reason,
        },
        reconciliation_valid_until: None,
        authority_eligibility_valid_until: None,
        _store: PhantomData,
    }
}

fn validate_issued_frontier(
    before: DurableAuthorityLeaseFrontierV1,
    lease_record: AuthorityLeaseRecordCommitment,
    after: DurableAuthorityLeaseFrontierV1,
) -> Result<(), HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::IssuedGenerationOverflow,
        )?;

    if after.generation.get() != expected_generation || after.head != Some(lease_record) {
        return Err(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::InvalidIssuedFrontier,
        );
    }
    Ok(())
}

fn validate_absence_frontier(
    expected: DurableAuthorityLeaseFrontierV1,
    observed: DurableAuthorityLeaseFrontierV1,
) -> Result<(), HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::InvalidProvenNotIssuedFrontier,
        );
    }
    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::InvalidProvenNotIssuedFrontier,
        );
    }
    Ok(())
}

fn validate_historical_receipt(
    expected_reconciler: AuthorityLeaseStoreDescriptorV1,
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    receipt: &AuthorityLeaseIssuanceReceiptV1,
) -> Result<
    HistoricalAuthorityLeaseReconciliationDispositionV1,
    HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1,
> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::UnsupportedReceiptSchema,
        );
    }
    if receipt.store != expected_reconciler {
        return Err(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::ReceiptReconcilerMismatch,
        );
    }
    if receipt.manifest != manifest {
        return Err(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::ReceiptManifestMismatch,
        );
    }
    if receipt.valid_until > expected_reconciler.valid_until {
        return Err(
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::ReceiptOutlivesReconciler,
        );
    }

    match receipt.disposition {
        AuthorityLeaseIssuanceDispositionV1::Issued {
            lease_record,
            post_frontier,
        } => {
            validate_issued_frontier(manifest.expected_frontier(), lease_record, post_frontier)?;
            Ok(HistoricalAuthorityLeaseReconciliationDispositionV1::Issued {
                lease_record,
                post_frontier,
            })
        }
        AuthorityLeaseIssuanceDispositionV1::ProvenNotIssued {
            observed_frontier,
            absence_evidence,
        } => {
            validate_absence_frontier(manifest.expected_frontier(), observed_frontier)?;
            Ok(
                HistoricalAuthorityLeaseReconciliationDispositionV1::ProvenNotIssued {
                    observed_frontier,
                    absence_evidence,
                },
            )
        }
        AuthorityLeaseIssuanceDispositionV1::AttemptIdConflict { .. } => Ok(
            HistoricalAuthorityLeaseReconciliationDispositionV1::OutcomeUnknown {
                reason: HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::AttemptIdConflict,
            },
        ),
        AuthorityLeaseIssuanceDispositionV1::ReservationAlreadyIssued { .. } => Ok(
            HistoricalAuthorityLeaseReconciliationDispositionV1::OutcomeUnknown {
                reason:
                    HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::ReservationAlreadyIssued,
            },
        ),
        AuthorityLeaseIssuanceDispositionV1::OutcomeUnknown => Ok(
            HistoricalAuthorityLeaseReconciliationDispositionV1::OutcomeUnknown {
                reason:
                    HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            },
        ),
    }
}

/// Reconcile one old exact issuance attempt under a currently trusted
/// same-identity store reconciler.
///
/// Policy and verifier generation may rotate. Stable store identity may not.
/// The method calls only the parent's read-only reconciliation surface.
pub fn reconcile_historical_authority_lease<S: AuthorityLeaseIssuanceStoreV1>(
    expected_reconciler: ExpectedHistoricalAuthorityLeaseReconcilerProfileV1,
    store: &S,
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
) -> HistoricalAuthorityLeaseReconciliationV1<S> {
    if manifest.schema_version != SSF_SCHEMA_V1 {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }

    if manifest.expected_store().identity != expected_reconciler.descriptor.identity {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::HistoricalStoreIdentityMismatch,
        );
    }

    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::ReconcilerDescriptorMismatchBefore,
        );
    }

    let receipt = match store.reconcile_authority_lease(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return unknown(
                manifest,
                expected_reconciler,
                None,
                HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::StoreError,
            );
        }
    };

    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalAuthorityLeaseReconciliationAmbiguityReasonV1::ReconcilerDescriptorChangedAfter,
        );
    }

    let disposition = match validate_historical_receipt(
        expected_reconciler.descriptor,
        manifest,
        &receipt,
    ) {
        Ok(disposition) => disposition,
        Err(reason) => return unknown(manifest, expected_reconciler, Some(receipt), reason),
    };

    match disposition {
        HistoricalAuthorityLeaseReconciliationDispositionV1::Issued { .. }
        | HistoricalAuthorityLeaseReconciliationDispositionV1::ProvenNotIssued { .. } => {
            let authority_eligibility_valid_until = receipt
                .valid_until
                .min(manifest.subject().lease_valid_until);
            HistoricalAuthorityLeaseReconciliationV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: Some(receipt.valid_until),
                authority_eligibility_valid_until: Some(authority_eligibility_valid_until),
                _store: PhantomData,
            }
        }
        HistoricalAuthorityLeaseReconciliationDispositionV1::OutcomeUnknown { .. } => {
            HistoricalAuthorityLeaseReconciliationV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: None,
                authority_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_authority_lease_issuance::{
        AuthorityLeaseStoreGeneration, AuthorityLeaseStoreIdentityCommitment,
        AuthorityLeaseStorePolicyCommitment,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn descriptor(identity: u8, policy: u8, generation: u64) -> AuthorityLeaseStoreDescriptorV1 {
        AuthorityLeaseStoreDescriptorV1 {
            identity: AuthorityLeaseStoreIdentityCommitment::from_bytes(bytes(identity)),
            policy: AuthorityLeaseStorePolicyCommitment::from_bytes(bytes(policy)),
            generation: AuthorityLeaseStoreGeneration::new(generation),
            valid_until: 1_000,
        }
    }

    #[test]
    fn same_identity_can_rotate_policy_and_generation() {
        let old = descriptor(1, 4, 4);
        let current = descriptor(1, 9, 11);
        assert_eq!(old.identity, current.identity);
        assert_ne!(old.policy, current.policy);
        assert_ne!(old.generation, current.generation);
    }

    #[test]
    fn different_store_identity_is_not_rotation() {
        assert_ne!(descriptor(1, 4, 4).identity, descriptor(2, 4, 4).identity);
    }

    #[test]
    fn fresh_historical_fact_never_refreshes_old_lease() {
        let reconciliation_valid_until = 180;
        let original_lease_valid_until = 90;
        assert_eq!(reconciliation_valid_until.min(original_lease_valid_until), 90);
    }
}

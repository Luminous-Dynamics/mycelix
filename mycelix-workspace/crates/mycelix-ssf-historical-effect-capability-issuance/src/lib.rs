// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only historical reconciliation for durable SSF effect-capability issuance.
//!
//! A capability-issuance attempt may remain unresolved longer than the store
//! policy/key generation that accepted the original write. A currently trusted
//! reconciler may attest that old exact attempt only under the same stable store
//! identity. Historical truth never refreshes the old operation-material lifetime
//! and never recreates a transient effect capability.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_durable_effect_capability_issuance::{
    DurableEffectCapabilityFrontierV1, EffectCapabilityIssuanceAbsenceCommitment,
    EffectCapabilityIssuanceAttemptManifestV1, EffectCapabilityIssuanceDispositionV1,
    EffectCapabilityIssuanceReceiptV1, EffectCapabilityIssuanceStoreDescriptorV1,
    EffectCapabilityIssuanceStoreV1, EffectCapabilityRecordCommitment,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedHistoricalEffectCapabilityReconcilerProfileV1 {
    descriptor: EffectCapabilityIssuanceStoreDescriptorV1,
}

impl ExpectedHistoricalEffectCapabilityReconcilerProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: EffectCapabilityIssuanceStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> EffectCapabilityIssuanceStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalEffectCapabilityAmbiguityReasonV1 {
    UnsupportedManifestSchema,
    HistoricalStoreIdentityMismatch,
    ReconcilerDescriptorMismatchBefore,
    ReconcilerDescriptorChangedAfter,
    StoreError,
    UnsupportedReceiptSchema,
    ReceiptReconcilerMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesReconciler,
    ReceiptOutlivesOriginalMaterial,
    IssuedGenerationOverflow,
    InvalidIssuedFrontier,
    InvalidProvenNotIssuedFrontier,
    AttemptIdConflict,
    EffectAdmissionAlreadyIssued,
    StoreReportedOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalEffectCapabilityDispositionV1 {
    Issued {
        capability_record: EffectCapabilityRecordCommitment,
        post_frontier: DurableEffectCapabilityFrontierV1,
    },
    ProvenNotIssued {
        observed_frontier: DurableEffectCapabilityFrontierV1,
        absence_evidence: EffectCapabilityIssuanceAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: HistoricalEffectCapabilityAmbiguityReasonV1,
    },
}

/// Read-only historical disposition under a currently trusted reconciler.
pub struct HistoricalEffectCapabilityIssuanceV1<S> {
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalEffectCapabilityReconcilerProfileV1,
    receipt: Option<EffectCapabilityIssuanceReceiptV1>,
    disposition: HistoricalEffectCapabilityDispositionV1,
    reconciliation_valid_until: Option<u64>,
    capability_eligibility_valid_until: Option<u64>,
    _store: PhantomData<fn() -> S>,
}

impl<S> HistoricalEffectCapabilityIssuanceV1<S> {
    pub const fn manifest(&self) -> EffectCapabilityIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_reconciler(
        &self,
    ) -> ExpectedHistoricalEffectCapabilityReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub fn receipt(&self) -> Option<&EffectCapabilityIssuanceReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> HistoricalEffectCapabilityDispositionV1 {
        self.disposition
    }

    pub const fn reconciliation_valid_until(&self) -> Option<u64> {
        self.reconciliation_valid_until
    }

    pub const fn capability_eligibility_valid_until(&self) -> Option<u64> {
        self.capability_eligibility_valid_until
    }

    pub fn permits_new_attempt(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalEffectCapabilityDispositionV1::ProvenNotIssued { .. }
        )
    }

    pub fn issued_lineage_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalEffectCapabilityDispositionV1::Issued { .. }
        )
    }

    pub const fn refreshes_original_material_authority(&self) -> bool {
        false
    }

    pub const fn recreates_source_owned_material(&self) -> bool {
        false
    }

    pub const fn contains_transient_effect_capability(&self) -> bool {
        false
    }
}

fn unknown<S>(
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalEffectCapabilityReconcilerProfileV1,
    receipt: Option<EffectCapabilityIssuanceReceiptV1>,
    reason: HistoricalEffectCapabilityAmbiguityReasonV1,
) -> HistoricalEffectCapabilityIssuanceV1<S> {
    HistoricalEffectCapabilityIssuanceV1 {
        manifest,
        expected_reconciler,
        receipt,
        disposition: HistoricalEffectCapabilityDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        capability_eligibility_valid_until: None,
        _store: PhantomData,
    }
}

fn validate_issued_frontier(
    before: DurableEffectCapabilityFrontierV1,
    capability_record: EffectCapabilityRecordCommitment,
    after: DurableEffectCapabilityFrontierV1,
) -> Result<(), HistoricalEffectCapabilityAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(HistoricalEffectCapabilityAmbiguityReasonV1::IssuedGenerationOverflow)?;

    if after.generation.get() != expected_generation || after.head != Some(capability_record) {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::InvalidIssuedFrontier);
    }
    Ok(())
}

fn validate_absence_frontier(
    expected: DurableEffectCapabilityFrontierV1,
    observed: DurableEffectCapabilityFrontierV1,
) -> Result<(), HistoricalEffectCapabilityAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::InvalidProvenNotIssuedFrontier);
    }
    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::InvalidProvenNotIssuedFrontier);
    }
    Ok(())
}

fn validate_historical_receipt(
    expected_reconciler: EffectCapabilityIssuanceStoreDescriptorV1,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    receipt: &EffectCapabilityIssuanceReceiptV1,
) -> Result<HistoricalEffectCapabilityDispositionV1, HistoricalEffectCapabilityAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_reconciler {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::ReceiptReconcilerMismatch);
    }
    if receipt.manifest != manifest {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_reconciler.valid_until {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::ReceiptOutlivesReconciler);
    }
    if receipt.valid_until > manifest.subject().material_valid_until {
        return Err(HistoricalEffectCapabilityAmbiguityReasonV1::ReceiptOutlivesOriginalMaterial);
    }

    match receipt.disposition {
        EffectCapabilityIssuanceDispositionV1::Issued {
            capability_record,
            post_frontier,
        } => {
            validate_issued_frontier(
                manifest.expected_frontier(),
                capability_record,
                post_frontier,
            )?;
            Ok(HistoricalEffectCapabilityDispositionV1::Issued {
                capability_record,
                post_frontier,
            })
        }
        EffectCapabilityIssuanceDispositionV1::ProvenNotIssued {
            observed_frontier,
            absence_evidence,
        } => {
            validate_absence_frontier(manifest.expected_frontier(), observed_frontier)?;
            Ok(HistoricalEffectCapabilityDispositionV1::ProvenNotIssued {
                observed_frontier,
                absence_evidence,
            })
        }
        EffectCapabilityIssuanceDispositionV1::AttemptIdConflict { .. } => Ok(
            HistoricalEffectCapabilityDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectCapabilityAmbiguityReasonV1::AttemptIdConflict,
            },
        ),
        EffectCapabilityIssuanceDispositionV1::EffectAdmissionAlreadyIssued { .. } => Ok(
            HistoricalEffectCapabilityDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectCapabilityAmbiguityReasonV1::EffectAdmissionAlreadyIssued,
            },
        ),
        EffectCapabilityIssuanceDispositionV1::OutcomeUnknown => Ok(
            HistoricalEffectCapabilityDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectCapabilityAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            },
        ),
    }
}

/// Reconcile one old exact capability-issuance attempt under a currently
/// trusted same-stable-identity store reconciler.
///
/// Policy/key/verifier generation may rotate. Stable store identity may not.
pub fn reconcile_historical_effect_capability_issuance<S>(
    expected_reconciler: ExpectedHistoricalEffectCapabilityReconcilerProfileV1,
    store: &S,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
) -> HistoricalEffectCapabilityIssuanceV1<S>
where
    S: EffectCapabilityIssuanceStoreV1,
{
    if manifest.schema_version != SSF_SCHEMA_V1 {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectCapabilityAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }

    if manifest.expected_store().stable_identity != expected_reconciler.descriptor.stable_identity {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectCapabilityAmbiguityReasonV1::HistoricalStoreIdentityMismatch,
        );
    }

    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectCapabilityAmbiguityReasonV1::ReconcilerDescriptorMismatchBefore,
        );
    }

    let receipt = match store.reconcile_effect_capability_lineage(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return unknown(
                manifest,
                expected_reconciler,
                None,
                HistoricalEffectCapabilityAmbiguityReasonV1::StoreError,
            );
        }
    };

    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalEffectCapabilityAmbiguityReasonV1::ReconcilerDescriptorChangedAfter,
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
        HistoricalEffectCapabilityDispositionV1::Issued { .. } => {
            let capability_eligibility_valid_until = receipt
                .valid_until
                .min(manifest.subject().material_valid_until);
            HistoricalEffectCapabilityIssuanceV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: Some(receipt.valid_until),
                capability_eligibility_valid_until: Some(capability_eligibility_valid_until),
                _store: PhantomData,
            }
        }
        HistoricalEffectCapabilityDispositionV1::ProvenNotIssued { .. } => {
            HistoricalEffectCapabilityIssuanceV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: Some(receipt.valid_until),
                capability_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
        HistoricalEffectCapabilityDispositionV1::OutcomeUnknown { .. } => {
            HistoricalEffectCapabilityIssuanceV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: None,
                capability_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_durable_effect_capability_issuance::{
        DurableEffectCapabilityGeneration, EffectCapabilityIssuanceStoreGeneration,
        EffectCapabilityIssuanceStoreIdentityCommitment,
        EffectCapabilityIssuanceStorePolicyCommitment, EffectCapabilityIssuanceStoreTimeBasisV1,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn descriptor(
        identity: u8,
        policy: u8,
        generation: u64,
    ) -> EffectCapabilityIssuanceStoreDescriptorV1 {
        EffectCapabilityIssuanceStoreDescriptorV1 {
            stable_identity: EffectCapabilityIssuanceStoreIdentityCommitment::from_bytes(bytes(identity)),
            policy: EffectCapabilityIssuanceStorePolicyCommitment::from_bytes(bytes(policy)),
            generation: EffectCapabilityIssuanceStoreGeneration::new(generation),
            time_basis: EffectCapabilityIssuanceStoreTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 1_000,
        }
    }

    fn frontier(generation: u64, head: Option<u8>) -> DurableEffectCapabilityFrontierV1 {
        DurableEffectCapabilityFrontierV1 {
            generation: DurableEffectCapabilityGeneration::new(generation),
            head: head.map(|byte| EffectCapabilityRecordCommitment::from_bytes(bytes(byte))),
        }
    }

    #[test]
    fn same_identity_can_rotate_policy_and_generation() {
        let old = descriptor(1, 4, 4);
        let current = descriptor(1, 9, 11);
        assert_eq!(old.stable_identity, current.stable_identity);
        assert_ne!(old.policy, current.policy);
        assert_ne!(old.generation, current.generation);
    }

    #[test]
    fn different_store_identity_is_not_rotation() {
        assert_ne!(
            descriptor(1, 4, 4).stable_identity,
            descriptor(2, 4, 4).stable_identity
        );
    }

    #[test]
    fn fresh_historical_fact_never_refreshes_original_material() {
        let reconciliation_valid_until = 180;
        let original_material_valid_until = 90;
        assert_eq!(
            reconciliation_valid_until.min(original_material_valid_until),
            90
        );
    }

    #[test]
    fn historical_issued_frontier_is_exact() {
        let record = EffectCapabilityRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            validate_issued_frontier(frontier(4, Some(1)), record, frontier(5, Some(9))),
            Ok(())
        );
    }

    #[test]
    fn historical_skipped_generation_is_rejected() {
        let record = EffectCapabilityRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            validate_issued_frontier(frontier(4, Some(1)), record, frontier(6, Some(9))),
            Err(HistoricalEffectCapabilityAmbiguityReasonV1::InvalidIssuedFrontier)
        );
    }
}

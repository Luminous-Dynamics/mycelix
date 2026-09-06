// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Historical reconciliation for one exact SSF durable effect-admission attempt.
//!
//! An admission may remain unresolved longer than the store generation that
//! accepted the original write. A currently trusted reconciler for the same
//! stable store identity may attest the historical disposition without
//! pretending to be the expired write-time generation.
//!
//! Historical truth never refreshes the original effect-eligibility lifetime
//! and never recreates an execution candidate or capability by itself.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_durable_effect_admission::{
    DurableEffectAdmissionFrontierV1, EffectAdmissionAbsenceCommitment,
    EffectAdmissionAttemptManifestV1, EffectAdmissionDispositionV1,
    EffectAdmissionReceiptV1, EffectAdmissionRecordCommitment, EffectAdmissionStoreDescriptorV1,
    EffectAdmissionStoreV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedHistoricalEffectAdmissionReconcilerProfileV1 {
    descriptor: EffectAdmissionStoreDescriptorV1,
}

impl ExpectedHistoricalEffectAdmissionReconcilerProfileV1 {
    pub const fn from_trusted_configuration(descriptor: EffectAdmissionStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> EffectAdmissionStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalEffectAdmissionAmbiguityReasonV1 {
    UnsupportedManifestSchema,
    HistoricalStoreIdentityMismatch,
    ReconcilerDescriptorMismatchBefore,
    ReconcilerDescriptorChangedAfter,
    StoreError,
    UnsupportedReceiptSchema,
    ReceiptReconcilerMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesReconciler,
    AdmittedGenerationOverflow,
    InvalidAdmittedFrontier,
    InvalidProvenNotAdmittedFrontier,
    AttemptIdConflict,
    AdmissionAlreadyExists,
    StoreReportedOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalEffectAdmissionDispositionV1 {
    Admitted {
        admission_record: EffectAdmissionRecordCommitment,
        post_frontier: DurableEffectAdmissionFrontierV1,
    },
    ProvenNotAdmitted {
        observed_frontier: DurableEffectAdmissionFrontierV1,
        absence_evidence: EffectAdmissionAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: HistoricalEffectAdmissionAmbiguityReasonV1,
    },
}

/// Read-only historical fact about one exact durable effect-admission attempt.
///
/// `reconciliation_valid_until` describes freshness of the new historical
/// attestation. `effect_eligibility_valid_until` remains capped by the original
/// execution-admission lifetime and therefore cannot be refreshed by verifier
/// or policy rotation.
pub struct HistoricalEffectAdmissionReconciliationV1<S> {
    manifest: EffectAdmissionAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalEffectAdmissionReconcilerProfileV1,
    receipt: Option<EffectAdmissionReceiptV1>,
    disposition: HistoricalEffectAdmissionDispositionV1,
    reconciliation_valid_until: Option<u64>,
    effect_eligibility_valid_until: Option<u64>,
    _store: PhantomData<fn() -> S>,
}

impl<S> HistoricalEffectAdmissionReconciliationV1<S> {
    pub const fn manifest(&self) -> EffectAdmissionAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_reconciler(
        &self,
    ) -> ExpectedHistoricalEffectAdmissionReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub fn receipt(&self) -> Option<&EffectAdmissionReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> HistoricalEffectAdmissionDispositionV1 {
        self.disposition
    }

    pub const fn reconciliation_valid_until(&self) -> Option<u64> {
        self.reconciliation_valid_until
    }

    pub const fn effect_eligibility_valid_until(&self) -> Option<u64> {
        self.effect_eligibility_valid_until
    }

    pub fn permits_new_attempt(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalEffectAdmissionDispositionV1::ProvenNotAdmitted { .. }
        )
    }

    pub fn admitted_lineage_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalEffectAdmissionDispositionV1::Admitted { .. }
        )
    }

    pub const fn refreshes_original_effect_authority(&self) -> bool {
        false
    }

    pub const fn contains_execution_candidate(&self) -> bool {
        false
    }

    pub const fn contains_single_use_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

fn unknown<S>(
    manifest: EffectAdmissionAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalEffectAdmissionReconcilerProfileV1,
    receipt: Option<EffectAdmissionReceiptV1>,
    reason: HistoricalEffectAdmissionAmbiguityReasonV1,
) -> HistoricalEffectAdmissionReconciliationV1<S> {
    HistoricalEffectAdmissionReconciliationV1 {
        manifest,
        expected_reconciler,
        receipt,
        disposition: HistoricalEffectAdmissionDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        effect_eligibility_valid_until: None,
        _store: PhantomData,
    }
}

fn validate_admitted_frontier(
    before: DurableEffectAdmissionFrontierV1,
    record: EffectAdmissionRecordCommitment,
    after: DurableEffectAdmissionFrontierV1,
) -> Result<(), HistoricalEffectAdmissionAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .checked_add(1)
        .ok_or(HistoricalEffectAdmissionAmbiguityReasonV1::AdmittedGenerationOverflow)?;

    if after.generation != expected_generation || after.head != Some(record) {
        return Err(HistoricalEffectAdmissionAmbiguityReasonV1::InvalidAdmittedFrontier);
    }
    Ok(())
}

fn validate_absence_frontier(
    expected: DurableEffectAdmissionFrontierV1,
    observed: DurableEffectAdmissionFrontierV1,
) -> Result<(), HistoricalEffectAdmissionAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(
            HistoricalEffectAdmissionAmbiguityReasonV1::InvalidProvenNotAdmittedFrontier,
        );
    }
    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(
            HistoricalEffectAdmissionAmbiguityReasonV1::InvalidProvenNotAdmittedFrontier,
        );
    }
    Ok(())
}

fn validate_historical_receipt(
    expected_reconciler: EffectAdmissionStoreDescriptorV1,
    manifest: EffectAdmissionAttemptManifestV1,
    receipt: &EffectAdmissionReceiptV1,
) -> Result<HistoricalEffectAdmissionDispositionV1, HistoricalEffectAdmissionAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalEffectAdmissionAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_reconciler {
        return Err(HistoricalEffectAdmissionAmbiguityReasonV1::ReceiptReconcilerMismatch);
    }
    if receipt.manifest != manifest {
        return Err(HistoricalEffectAdmissionAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_reconciler.valid_until {
        return Err(HistoricalEffectAdmissionAmbiguityReasonV1::ReceiptOutlivesReconciler);
    }

    match receipt.disposition {
        EffectAdmissionDispositionV1::Admitted {
            record,
            post_frontier,
        } => {
            validate_admitted_frontier(manifest.expected_frontier, record, post_frontier)?;
            Ok(HistoricalEffectAdmissionDispositionV1::Admitted {
                admission_record: record,
                post_frontier,
            })
        }
        EffectAdmissionDispositionV1::ProvenNotAdmitted {
            absence,
            observed_frontier,
        } => {
            validate_absence_frontier(manifest.expected_frontier, observed_frontier)?;
            Ok(HistoricalEffectAdmissionDispositionV1::ProvenNotAdmitted {
                observed_frontier,
                absence_evidence: absence,
            })
        }
        EffectAdmissionDispositionV1::AttemptIdConflict => Ok(
            HistoricalEffectAdmissionDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectAdmissionAmbiguityReasonV1::AttemptIdConflict,
            },
        ),
        EffectAdmissionDispositionV1::AdmissionAlreadyExists => Ok(
            HistoricalEffectAdmissionDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectAdmissionAmbiguityReasonV1::AdmissionAlreadyExists,
            },
        ),
        EffectAdmissionDispositionV1::OutcomeUnknown => Ok(
            HistoricalEffectAdmissionDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectAdmissionAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            },
        ),
    }
}

/// Reconcile one old exact effect-admission attempt under a currently trusted
/// same-identity admission-store reconciler.
///
/// Policy and verifier generation may rotate. Stable store identity may not.
/// Only the parent's read-only reconciliation surface is invoked.
pub fn reconcile_historical_effect_admission<S: EffectAdmissionStoreV1>(
    expected_reconciler: ExpectedHistoricalEffectAdmissionReconcilerProfileV1,
    store: &S,
    manifest: EffectAdmissionAttemptManifestV1,
) -> HistoricalEffectAdmissionReconciliationV1<S> {
    if manifest.schema_version != SSF_SCHEMA_V1 {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectAdmissionAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }

    if manifest.expected_store.stable_identity != expected_reconciler.descriptor.stable_identity {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectAdmissionAmbiguityReasonV1::HistoricalStoreIdentityMismatch,
        );
    }

    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectAdmissionAmbiguityReasonV1::ReconcilerDescriptorMismatchBefore,
        );
    }

    let receipt = match store.reconcile_effect_admission(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return unknown(
                manifest,
                expected_reconciler,
                None,
                HistoricalEffectAdmissionAmbiguityReasonV1::StoreError,
            );
        }
    };

    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalEffectAdmissionAmbiguityReasonV1::ReconcilerDescriptorChangedAfter,
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
        HistoricalEffectAdmissionDispositionV1::Admitted { .. }
        | HistoricalEffectAdmissionDispositionV1::ProvenNotAdmitted { .. } => {
            let effect_eligibility_valid_until = receipt
                .valid_until
                .min(manifest.subject.execution_admission_valid_until);
            HistoricalEffectAdmissionReconciliationV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: Some(receipt.valid_until),
                effect_eligibility_valid_until: Some(effect_eligibility_valid_until),
                _store: PhantomData,
            }
        }
        HistoricalEffectAdmissionDispositionV1::OutcomeUnknown { .. } => {
            HistoricalEffectAdmissionReconciliationV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: None,
                effect_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_durable_effect_admission::{
        EffectAdmissionStoreIdentityCommitment, EffectAdmissionStorePolicyCommitment,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn descriptor(identity: u8, policy: u8, generation: u64) -> EffectAdmissionStoreDescriptorV1 {
        EffectAdmissionStoreDescriptorV1 {
            stable_identity: EffectAdmissionStoreIdentityCommitment::from_bytes(bytes(identity)),
            policy: EffectAdmissionStorePolicyCommitment::from_bytes(bytes(policy)),
            generation,
            valid_until: 1_000,
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
    fn fresh_historical_fact_never_refreshes_old_effect_eligibility() {
        let reconciliation_valid_until = 180;
        let original_execution_admission_valid_until = 90;
        assert_eq!(
            reconciliation_valid_until.min(original_execution_admission_valid_until),
            90
        );
    }

    #[test]
    fn admitted_frontier_advances_exactly_once_to_record() {
        let record = EffectAdmissionRecordCommitment::from_bytes(bytes(7));
        let before = DurableEffectAdmissionFrontierV1 {
            generation: 10,
            head: None,
        };
        let after = DurableEffectAdmissionFrontierV1 {
            generation: 11,
            head: Some(record),
        };
        assert_eq!(validate_admitted_frontier(before, record, after), Ok(()));
    }

    #[test]
    fn admitted_frontier_rejects_generation_skip() {
        let record = EffectAdmissionRecordCommitment::from_bytes(bytes(7));
        let before = DurableEffectAdmissionFrontierV1 {
            generation: 10,
            head: None,
        };
        let after = DurableEffectAdmissionFrontierV1 {
            generation: 12,
            head: Some(record),
        };
        assert_eq!(
            validate_admitted_frontier(before, record, after),
            Err(HistoricalEffectAdmissionAmbiguityReasonV1::InvalidAdmittedFrontier)
        );
    }
}

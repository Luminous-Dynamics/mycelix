// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only historical reconciliation for durable SSF effect-capability claims.
//!
//! A durable claim may remain unresolved longer than the claim-store policy/key
//! generation that accepted it. A currently trusted same-identity reconciler may
//! attest that old exact attempt without creating a new claim or external effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_durable_effect_capability_claim::{
    DurableEffectCapabilityClaimFrontierV1, EffectCapabilityClaimAbsenceCommitment,
    EffectCapabilityClaimAttemptManifestV1, EffectCapabilityClaimDispositionV1,
    EffectCapabilityClaimReceiptV1, EffectCapabilityClaimRecordCommitment,
    EffectCapabilityClaimStoreDescriptorV1, EffectCapabilityClaimStoreV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedHistoricalEffectCapabilityClaimReconcilerProfileV1 {
    descriptor: EffectCapabilityClaimStoreDescriptorV1,
}

impl ExpectedHistoricalEffectCapabilityClaimReconcilerProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: EffectCapabilityClaimStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> EffectCapabilityClaimStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalEffectCapabilityClaimAmbiguityReasonV1 {
    UnsupportedManifestSchema,
    HistoricalStoreIdentityMismatch,
    ReconcilerDescriptorMismatchBefore,
    ReconcilerDescriptorChangedAfter,
    StoreError,
    UnsupportedReceiptSchema,
    ReceiptReconcilerMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesReconciler,
    ReceiptOutlivesOriginalCapability,
    ReceiptOutlivesOriginalClaimTime,
    ClaimedGenerationOverflow,
    InvalidClaimedFrontier,
    InvalidProvenNotClaimedFrontier,
    AttemptIdConflict,
    CapabilityAlreadyClaimed,
    StoreReportedOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalEffectCapabilityClaimDispositionV1 {
    Claimed {
        claim_record: EffectCapabilityClaimRecordCommitment,
        post_frontier: DurableEffectCapabilityClaimFrontierV1,
    },
    ProvenNotClaimed {
        observed_frontier: DurableEffectCapabilityClaimFrontierV1,
        absence_evidence: EffectCapabilityClaimAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: HistoricalEffectCapabilityClaimAmbiguityReasonV1,
    },
}

pub struct HistoricalEffectCapabilityClaimV1<S> {
    manifest: EffectCapabilityClaimAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalEffectCapabilityClaimReconcilerProfileV1,
    receipt: Option<EffectCapabilityClaimReceiptV1>,
    disposition: HistoricalEffectCapabilityClaimDispositionV1,
    reconciliation_valid_until: Option<u64>,
    claim_eligibility_valid_until: Option<u64>,
    _store: PhantomData<fn() -> S>,
}

impl<S> HistoricalEffectCapabilityClaimV1<S> {
    pub const fn manifest(&self) -> EffectCapabilityClaimAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_reconciler(
        &self,
    ) -> ExpectedHistoricalEffectCapabilityClaimReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub fn receipt(&self) -> Option<&EffectCapabilityClaimReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> HistoricalEffectCapabilityClaimDispositionV1 {
        self.disposition
    }

    pub const fn reconciliation_valid_until(&self) -> Option<u64> {
        self.reconciliation_valid_until
    }

    pub const fn claim_eligibility_valid_until(&self) -> Option<u64> {
        self.claim_eligibility_valid_until
    }

    pub fn permits_new_claim_attempt(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalEffectCapabilityClaimDispositionV1::ProvenNotClaimed { .. }
        )
    }

    pub fn claimed_lineage_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalEffectCapabilityClaimDispositionV1::Claimed { .. }
        )
    }

    pub const fn reconstructs_transient_capability(&self) -> bool {
        false
    }

    pub const fn provider_handle_resolved(&self) -> bool {
        false
    }

    pub const fn actuator_invocation_performed(&self) -> bool {
        false
    }
}

fn unknown<S>(
    manifest: EffectCapabilityClaimAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalEffectCapabilityClaimReconcilerProfileV1,
    receipt: Option<EffectCapabilityClaimReceiptV1>,
    reason: HistoricalEffectCapabilityClaimAmbiguityReasonV1,
) -> HistoricalEffectCapabilityClaimV1<S> {
    HistoricalEffectCapabilityClaimV1 {
        manifest,
        expected_reconciler,
        receipt,
        disposition: HistoricalEffectCapabilityClaimDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        claim_eligibility_valid_until: None,
        _store: PhantomData,
    }
}

fn validate_claimed_frontier(
    before: DurableEffectCapabilityClaimFrontierV1,
    claim_record: EffectCapabilityClaimRecordCommitment,
    after: DurableEffectCapabilityClaimFrontierV1,
) -> Result<(), HistoricalEffectCapabilityClaimAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(HistoricalEffectCapabilityClaimAmbiguityReasonV1::ClaimedGenerationOverflow)?;

    if after.generation.get() != expected_generation || after.head != Some(claim_record) {
        return Err(HistoricalEffectCapabilityClaimAmbiguityReasonV1::InvalidClaimedFrontier);
    }
    Ok(())
}

fn validate_absence_frontier(
    expected: DurableEffectCapabilityClaimFrontierV1,
    observed: DurableEffectCapabilityClaimFrontierV1,
) -> Result<(), HistoricalEffectCapabilityClaimAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::InvalidProvenNotClaimedFrontier,
        );
    }
    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::InvalidProvenNotClaimedFrontier,
        );
    }
    Ok(())
}

fn validate_receipt(
    expected_reconciler: EffectCapabilityClaimStoreDescriptorV1,
    manifest: EffectCapabilityClaimAttemptManifestV1,
    receipt: &EffectCapabilityClaimReceiptV1,
) -> Result<HistoricalEffectCapabilityClaimDispositionV1, HistoricalEffectCapabilityClaimAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalEffectCapabilityClaimAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_reconciler {
        return Err(HistoricalEffectCapabilityClaimAmbiguityReasonV1::ReceiptReconcilerMismatch);
    }
    if receipt.manifest != manifest {
        return Err(HistoricalEffectCapabilityClaimAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_reconciler.valid_until {
        return Err(HistoricalEffectCapabilityClaimAmbiguityReasonV1::ReceiptOutlivesReconciler);
    }
    if receipt.valid_until > manifest.subject().capability.valid_until {
        return Err(
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::ReceiptOutlivesOriginalCapability,
        );
    }
    if receipt.valid_until > manifest.subject().claim_time_valid_until {
        return Err(
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::ReceiptOutlivesOriginalClaimTime,
        );
    }

    match receipt.disposition {
        EffectCapabilityClaimDispositionV1::Claimed {
            claim_record,
            post_frontier,
        } => {
            validate_claimed_frontier(manifest.expected_frontier(), claim_record, post_frontier)?;
            Ok(HistoricalEffectCapabilityClaimDispositionV1::Claimed {
                claim_record,
                post_frontier,
            })
        }
        EffectCapabilityClaimDispositionV1::ProvenNotClaimed {
            observed_frontier,
            absence_evidence,
        } => {
            validate_absence_frontier(manifest.expected_frontier(), observed_frontier)?;
            Ok(HistoricalEffectCapabilityClaimDispositionV1::ProvenNotClaimed {
                observed_frontier,
                absence_evidence,
            })
        }
        EffectCapabilityClaimDispositionV1::AttemptIdConflict { .. } => Ok(
            HistoricalEffectCapabilityClaimDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectCapabilityClaimAmbiguityReasonV1::AttemptIdConflict,
            },
        ),
        EffectCapabilityClaimDispositionV1::CapabilityAlreadyClaimed { .. } => Ok(
            HistoricalEffectCapabilityClaimDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectCapabilityClaimAmbiguityReasonV1::CapabilityAlreadyClaimed,
            },
        ),
        EffectCapabilityClaimDispositionV1::OutcomeUnknown => Ok(
            HistoricalEffectCapabilityClaimDispositionV1::OutcomeUnknown {
                reason: HistoricalEffectCapabilityClaimAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            },
        ),
    }
}

pub fn reconcile_historical_effect_capability_claim<S>(
    expected_reconciler: ExpectedHistoricalEffectCapabilityClaimReconcilerProfileV1,
    store: &S,
    manifest: EffectCapabilityClaimAttemptManifestV1,
) -> HistoricalEffectCapabilityClaimV1<S>
where
    S: EffectCapabilityClaimStoreV1,
{
    if manifest.schema_version != SSF_SCHEMA_V1 {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }
    if manifest.expected_store().stable_identity != expected_reconciler.descriptor.stable_identity {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::HistoricalStoreIdentityMismatch,
        );
    }
    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::ReconcilerDescriptorMismatchBefore,
        );
    }

    let receipt = match store.reconcile_effect_capability_claim(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return unknown(
                manifest,
                expected_reconciler,
                None,
                HistoricalEffectCapabilityClaimAmbiguityReasonV1::StoreError,
            );
        }
    };

    if store.descriptor() != expected_reconciler.descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalEffectCapabilityClaimAmbiguityReasonV1::ReconcilerDescriptorChangedAfter,
        );
    }

    let disposition = match validate_receipt(expected_reconciler.descriptor, manifest, &receipt) {
        Ok(disposition) => disposition,
        Err(reason) => return unknown(manifest, expected_reconciler, Some(receipt), reason),
    };

    match disposition {
        HistoricalEffectCapabilityClaimDispositionV1::Claimed { .. } => {
            let claim_eligibility_valid_until = receipt
                .valid_until
                .min(manifest.subject().capability.valid_until)
                .min(manifest.subject().claim_time_valid_until);
            HistoricalEffectCapabilityClaimV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: Some(receipt.valid_until),
                claim_eligibility_valid_until: Some(claim_eligibility_valid_until),
                _store: PhantomData,
            }
        }
        HistoricalEffectCapabilityClaimDispositionV1::ProvenNotClaimed { .. } => {
            HistoricalEffectCapabilityClaimV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: Some(receipt.valid_until),
                claim_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
        HistoricalEffectCapabilityClaimDispositionV1::OutcomeUnknown { .. } => {
            HistoricalEffectCapabilityClaimV1 {
                manifest,
                expected_reconciler,
                receipt: Some(receipt),
                disposition,
                reconciliation_valid_until: None,
                claim_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_durable_effect_capability_claim::{
        DurableEffectCapabilityClaimGeneration, EffectCapabilityClaimStoreGeneration,
        EffectCapabilityClaimStoreIdentityCommitment, EffectCapabilityClaimStorePolicyCommitment,
        EffectCapabilityClaimStoreTimeBasisV1,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn descriptor(
        identity: u8,
        policy: u8,
        generation: u64,
    ) -> EffectCapabilityClaimStoreDescriptorV1 {
        EffectCapabilityClaimStoreDescriptorV1 {
            stable_identity: EffectCapabilityClaimStoreIdentityCommitment::from_bytes(bytes(identity)),
            policy: EffectCapabilityClaimStorePolicyCommitment::from_bytes(bytes(policy)),
            generation: EffectCapabilityClaimStoreGeneration::new(generation),
            time_basis: EffectCapabilityClaimStoreTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 1_000,
        }
    }

    fn frontier(generation: u64, head: Option<u8>) -> DurableEffectCapabilityClaimFrontierV1 {
        DurableEffectCapabilityClaimFrontierV1 {
            generation: DurableEffectCapabilityClaimGeneration::new(generation),
            head: head.map(|byte| EffectCapabilityClaimRecordCommitment::from_bytes(bytes(byte))),
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
    fn fresh_historical_fact_never_refreshes_original_claim_authority() {
        let reconciliation_valid_until = 180;
        let original_capability_valid_until = 100;
        let original_claim_time_valid_until = 90;
        assert_eq!(
            reconciliation_valid_until
                .min(original_capability_valid_until)
                .min(original_claim_time_valid_until),
            90
        );
    }

    #[test]
    fn historical_claimed_frontier_is_exact() {
        let record = EffectCapabilityClaimRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            validate_claimed_frontier(frontier(4, Some(1)), record, frontier(5, Some(9))),
            Ok(())
        );
    }

    #[test]
    fn historical_skipped_generation_is_rejected() {
        let record = EffectCapabilityClaimRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            validate_claimed_frontier(frontier(4, Some(1)), record, frontier(6, Some(9))),
            Err(HistoricalEffectCapabilityClaimAmbiguityReasonV1::InvalidClaimedFrontier)
        );
    }
}

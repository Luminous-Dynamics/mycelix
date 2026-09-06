// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Historical reconciliation for SSF adoption-authority registration.
//!
//! A registration attempt can remain unresolved longer than the verifier/key
//! generation that accepted the original write. This crate lets a currently
//! trusted reconciler attest the historical disposition of that exact attempt
//! without pretending to be the expired write-time generation.
//!
//! Historical reconciliation never refreshes the authority lifetime of the
//! original candidate. A freshly verified historical fact may therefore have a
//! later reconciliation-validity ceiling while its authority-eligibility
//! ceiling remains capped by the original candidate.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_adoption_authority_registration::{
    AdoptionAuthorityRegistrationReceiptV1, AdoptionAuthorityRegistrationStoreV1,
    DurableRegistrationFrontierV1, RegistrationAttemptAbsenceCommitment,
    RegistrationAttemptManifestV1, RegistrationDispositionV1, RegistrationRecordCommitment,
    RegistrationStoreDescriptorV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;

/// Independently trusted current verifier for historical registration facts.
///
/// v0.1 permits verifier generation and store policy rotation, but requires the
/// stable store identity to remain the same as the write-time manifest. Store
/// identity migration needs a separate migration proof and is intentionally
/// outside this contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedHistoricalRegistrationReconcilerProfileV1 {
    descriptor: RegistrationStoreDescriptorV1,
}

impl ExpectedHistoricalRegistrationReconcilerProfileV1 {
    pub const fn from_trusted_configuration(descriptor: RegistrationStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> RegistrationStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalReconciliationAmbiguityReasonV1 {
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
    InvalidProvenNotPersistedFrontier,
    AttemptIdConflict,
    StoreReportedOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalReconciliationDispositionV1 {
    Persisted {
        record: RegistrationRecordCommitment,
        post_frontier: DurableRegistrationFrontierV1,
    },
    ProvenNotPersisted {
        observed_frontier: DurableRegistrationFrontierV1,
        absence_evidence: RegistrationAttemptAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: HistoricalReconciliationAmbiguityReasonV1,
    },
}

/// Non-authoritative historical disposition evidence.
///
/// `reconciliation_valid_until` answers how long the current historical
/// attestation is qualified. `authority_eligibility_valid_until` is separately
/// capped by the original candidate and can never be refreshed by historical
/// re-attestation.
pub struct HistoricalRegistrationReconciliationV1<S> {
    manifest: RegistrationAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalRegistrationReconcilerProfileV1,
    receipt: Option<AdoptionAuthorityRegistrationReceiptV1>,
    disposition: HistoricalReconciliationDispositionV1,
    reconciliation_valid_until: Option<u64>,
    authority_eligibility_valid_until: Option<u64>,
    _store_type: PhantomData<fn() -> S>,
}

impl<S> HistoricalRegistrationReconciliationV1<S> {
    pub const fn manifest(&self) -> RegistrationAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_reconciler(
        &self,
    ) -> ExpectedHistoricalRegistrationReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub fn receipt(&self) -> Option<&AdoptionAuthorityRegistrationReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> HistoricalReconciliationDispositionV1 {
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
            HistoricalReconciliationDispositionV1::ProvenNotPersisted { .. }
        )
    }

    pub fn persisted_candidate_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            HistoricalReconciliationDispositionV1::Persisted { .. }
        )
    }

    pub const fn refreshes_original_candidate_authority(&self) -> bool {
        false
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

fn unknown<S>(
    manifest: RegistrationAttemptManifestV1,
    expected_reconciler: ExpectedHistoricalRegistrationReconcilerProfileV1,
    receipt: Option<AdoptionAuthorityRegistrationReceiptV1>,
    reason: HistoricalReconciliationAmbiguityReasonV1,
) -> HistoricalRegistrationReconciliationV1<S> {
    HistoricalRegistrationReconciliationV1 {
        manifest,
        expected_reconciler,
        receipt,
        disposition: HistoricalReconciliationDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        authority_eligibility_valid_until: None,
        _store_type: PhantomData,
    }
}

fn validate_persisted_frontier(
    before: DurableRegistrationFrontierV1,
    record: RegistrationRecordCommitment,
    after: DurableRegistrationFrontierV1,
) -> Result<(), HistoricalReconciliationAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(HistoricalReconciliationAmbiguityReasonV1::PersistedGenerationOverflow)?;

    if after.generation.get() != expected_generation || after.head != Some(record) {
        return Err(HistoricalReconciliationAmbiguityReasonV1::InvalidPersistedFrontier);
    }

    Ok(())
}

fn validate_absence_frontier(
    expected: DurableRegistrationFrontierV1,
    observed: DurableRegistrationFrontierV1,
) -> Result<(), HistoricalReconciliationAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(
            HistoricalReconciliationAmbiguityReasonV1::InvalidProvenNotPersistedFrontier,
        );
    }

    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(
            HistoricalReconciliationAmbiguityReasonV1::InvalidProvenNotPersistedFrontier,
        );
    }

    Ok(())
}

fn validate_historical_receipt(
    expected_reconciler: RegistrationStoreDescriptorV1,
    manifest: RegistrationAttemptManifestV1,
    receipt: &AdoptionAuthorityRegistrationReceiptV1,
) -> Result<HistoricalReconciliationDispositionV1, HistoricalReconciliationAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalReconciliationAmbiguityReasonV1::UnsupportedReceiptSchema);
    }

    if receipt.store != expected_reconciler {
        return Err(HistoricalReconciliationAmbiguityReasonV1::ReceiptReconcilerMismatch);
    }

    if receipt.manifest != manifest {
        return Err(HistoricalReconciliationAmbiguityReasonV1::ReceiptManifestMismatch);
    }

    if receipt.valid_until > expected_reconciler.valid_until {
        return Err(HistoricalReconciliationAmbiguityReasonV1::ReceiptOutlivesReconciler);
    }

    match receipt.disposition {
        RegistrationDispositionV1::Persisted {
            record,
            post_frontier,
        } => {
            validate_persisted_frontier(manifest.expected_frontier(), record, post_frontier)?;
            Ok(HistoricalReconciliationDispositionV1::Persisted {
                record,
                post_frontier,
            })
        }
        RegistrationDispositionV1::ProvenNotPersisted {
            observed_frontier,
            absence_evidence,
        } => {
            validate_absence_frontier(manifest.expected_frontier(), observed_frontier)?;
            Ok(HistoricalReconciliationDispositionV1::ProvenNotPersisted {
                observed_frontier,
                absence_evidence,
            })
        }
        RegistrationDispositionV1::AttemptIdConflict { .. } => {
            Ok(HistoricalReconciliationDispositionV1::OutcomeUnknown {
                reason: HistoricalReconciliationAmbiguityReasonV1::AttemptIdConflict,
            })
        }
        RegistrationDispositionV1::OutcomeUnknown => {
            Ok(HistoricalReconciliationDispositionV1::OutcomeUnknown {
                reason: HistoricalReconciliationAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            })
        }
    }
}

/// Reconcile one exact historical registration attempt using a currently
/// trusted reconciler under the same stable store identity.
///
/// This function calls only the parent's read-only `reconcile_attempt` surface.
/// The current reconciler may have a different policy and verifier generation
/// from the write-time store profile.
///
/// Historical re-attestation freshness is never allowed to revive stale
/// authority: the authority-eligibility ceiling is always
/// `min(reconciliation_receipt, original_candidate)`.
pub fn reconcile_historical_registration_attempt<S>(
    expected_reconciler: ExpectedHistoricalRegistrationReconcilerProfileV1,
    store: &S,
    manifest: RegistrationAttemptManifestV1,
) -> HistoricalRegistrationReconciliationV1<S>
where
    S: AdoptionAuthorityRegistrationStoreV1,
{
    if manifest.schema_version() != SSF_SCHEMA_V1 {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalReconciliationAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }

    let expected_descriptor = expected_reconciler.descriptor();
    if expected_descriptor.identity != manifest.expected_store().identity {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalReconciliationAmbiguityReasonV1::HistoricalStoreIdentityMismatch,
        );
    }

    if store.descriptor() != expected_descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            None,
            HistoricalReconciliationAmbiguityReasonV1::ReconcilerDescriptorMismatchBefore,
        );
    }

    let receipt = store.reconcile_attempt(&manifest);
    if store.descriptor() != expected_descriptor {
        return unknown(
            manifest,
            expected_reconciler,
            Some(receipt),
            HistoricalReconciliationAmbiguityReasonV1::ReconcilerDescriptorChangedAfter,
        );
    }

    let disposition = match validate_historical_receipt(expected_descriptor, manifest, &receipt) {
        Ok(value) => value,
        Err(reason) => {
            return unknown(manifest, expected_reconciler, Some(receipt), reason);
        }
    };

    let reconciliation_valid_until = Some(receipt.valid_until);
    let authority_eligibility_valid_until = Some(
        receipt
            .valid_until
            .min(manifest.subject().candidate_valid_until),
    );

    HistoricalRegistrationReconciliationV1 {
        manifest,
        expected_reconciler,
        receipt: Some(receipt),
        disposition,
        reconciliation_valid_until,
        authority_eligibility_valid_until,
        _store_type: PhantomData,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_adoption_authority_candidate::{
        AdoptionAuthorityCandidateId, AdoptionAuthorityNonce, AdoptionAuthorityOperationV1,
    };
    use mycelix_ssf_adoption_authority_registration::{
        DurableRegistrationGeneration, RegistrationAttemptId, RegistrationReceiptCommitment,
        RegistrationStoreIdentityCommitment, RegistrationStorePolicyCommitment,
        RegistrationStoreVerifierGeneration,
    };
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, ConsequenceClass, EvidenceCommitment,
        FederationEpoch, FederationId, PolicyDigest, RevocationGeneration,
    };
    use mycelix_ssf_local_adoption_policy::{
        LocalAdoptionScopeCommitment, PolicyEvaluationReceiptCommitment,
    };
    use mycelix_ssf_snapshots::{
        assemble_federation_state_head, FederationStateHeadV1, MembershipSnapshotV1,
        PolicySnapshotV1, RevocationSnapshotV1, SnapshotCommitment, SnapshotGeneration,
        SnapshotLineageV1,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn state(head_base: u8) -> FederationStateHeadV1 {
        let membership = MembershipSnapshotV1::from_lineage(
            SnapshotLineageV1::new(
                FederationId::from_bytes(bytes(1)),
                FederationEpoch::new(7),
                AuthorityRootCommitment::from_bytes(bytes(2)),
                AuthorityGeneration::new(11),
                SnapshotGeneration::new(0),
                None,
                SnapshotCommitment::from_bytes(bytes(head_base)),
                EvidenceCommitment::from_bytes(bytes(head_base.wrapping_add(40))),
            )
            .expect("membership"),
        );

        let revocation = RevocationSnapshotV1::from_lineage(
            SnapshotLineageV1::new(
                FederationId::from_bytes(bytes(1)),
                FederationEpoch::new(7),
                AuthorityRootCommitment::from_bytes(bytes(2)),
                AuthorityGeneration::new(11),
                SnapshotGeneration::new(0),
                None,
                SnapshotCommitment::from_bytes(bytes(head_base.wrapping_add(1))),
                EvidenceCommitment::from_bytes(bytes(head_base.wrapping_add(41))),
            )
            .expect("revocation"),
            RevocationGeneration::new(40),
        );

        let policy = PolicySnapshotV1::from_lineage(
            SnapshotLineageV1::new(
                FederationId::from_bytes(bytes(1)),
                FederationEpoch::new(7),
                AuthorityRootCommitment::from_bytes(bytes(2)),
                AuthorityGeneration::new(11),
                SnapshotGeneration::new(0),
                None,
                SnapshotCommitment::from_bytes(bytes(head_base.wrapping_add(2))),
                EvidenceCommitment::from_bytes(bytes(head_base.wrapping_add(42))),
            )
            .expect("policy"),
            PolicyDigest::from_bytes(bytes(3)),
        );

        assemble_federation_state_head(&membership, &revocation, &policy).expect("coherent")
    }

    fn write_descriptor() -> RegistrationStoreDescriptorV1 {
        RegistrationStoreDescriptorV1 {
            identity: RegistrationStoreIdentityCommitment::from_bytes(bytes(10)),
            policy: RegistrationStorePolicyCommitment::from_bytes(bytes(11)),
            verifier_generation: RegistrationStoreVerifierGeneration::new(4),
            valid_until: 100,
        }
    }

    fn rotated_descriptor() -> RegistrationStoreDescriptorV1 {
        RegistrationStoreDescriptorV1 {
            identity: write_descriptor().identity,
            policy: RegistrationStorePolicyCommitment::from_bytes(bytes(12)),
            verifier_generation: RegistrationStoreVerifierGeneration::new(5),
            valid_until: 200,
        }
    }

    fn foreign_descriptor() -> RegistrationStoreDescriptorV1 {
        RegistrationStoreDescriptorV1 {
            identity: RegistrationStoreIdentityCommitment::from_bytes(bytes(99)),
            policy: RegistrationStorePolicyCommitment::from_bytes(bytes(12)),
            verifier_generation: RegistrationStoreVerifierGeneration::new(5),
            valid_until: 200,
        }
    }

    fn frontier(generation: u64, head: Option<u8>) -> DurableRegistrationFrontierV1 {
        DurableRegistrationFrontierV1 {
            generation: DurableRegistrationGeneration::new(generation),
            head: head.map(|byte| RegistrationRecordCommitment::from_bytes(bytes(byte))),
        }
    }

    fn manifest() -> RegistrationAttemptManifestV1 {
        let subject = mycelix_ssf_adoption_authority_registration::AdoptionAuthorityRegistrationSubjectV1 {
            candidate_id: AdoptionAuthorityCandidateId::from_bytes(bytes(20)),
            nonce: AdoptionAuthorityNonce::from_bytes(bytes(21)),
            operation: AdoptionAuthorityOperationV1::InstallQualifiedRemoteState,
            source_local_state: state(30),
            remote_target: state(40),
            policy_receipt: PolicyEvaluationReceiptCommitment::from_bytes(bytes(22)),
            scope: LocalAdoptionScopeCommitment::from_bytes(bytes(23)),
            consequence: ConsequenceClass::C3ScopedRestriction,
            candidate_valid_until: 90,
        };

        RegistrationAttemptManifestV1::from_journaled_parts(
            RegistrationAttemptId::from_bytes(bytes(24)),
            subject,
            write_descriptor(),
            frontier(7, Some(3)),
        )
    }

    #[derive(Clone, Copy)]
    struct HistoricalStore {
        descriptor: RegistrationStoreDescriptorV1,
        disposition: RegistrationDispositionV1,
        receipt_valid_until: u64,
    }

    impl AdoptionAuthorityRegistrationStoreV1 for HistoricalStore {
        fn descriptor(&self) -> RegistrationStoreDescriptorV1 {
            self.descriptor
        }

        fn register_candidate(
            &self,
            manifest: &RegistrationAttemptManifestV1,
        ) -> AdoptionAuthorityRegistrationReceiptV1 {
            self.reconcile_attempt(manifest)
        }

        fn reconcile_attempt(
            &self,
            manifest: &RegistrationAttemptManifestV1,
        ) -> AdoptionAuthorityRegistrationReceiptV1 {
            AdoptionAuthorityRegistrationReceiptV1::new(
                self.descriptor,
                *manifest,
                self.disposition,
                self.receipt_valid_until,
                RegistrationReceiptCommitment::from_bytes(bytes(88)),
            )
        }
    }

    #[test]
    fn rotated_reconciler_can_attest_old_attempt_under_same_store_identity() {
        let store = HistoricalStore {
            descriptor: rotated_descriptor(),
            disposition: RegistrationDispositionV1::Persisted {
                record: RegistrationRecordCommitment::from_bytes(bytes(4)),
                post_frontier: frontier(8, Some(4)),
            },
            receipt_valid_until: 150,
        };

        let result = reconcile_historical_registration_attempt(
            ExpectedHistoricalRegistrationReconcilerProfileV1::from_trusted_configuration(
                rotated_descriptor(),
            ),
            &store,
            manifest(),
        );

        assert!(result.persisted_candidate_may_be_rebound());
        assert_eq!(result.reconciliation_valid_until(), Some(150));
        assert_eq!(result.authority_eligibility_valid_until(), Some(90));
        assert!(!result.refreshes_original_candidate_authority());
    }

    #[test]
    fn different_store_identity_cannot_reconcile_old_attempt() {
        let store = HistoricalStore {
            descriptor: foreign_descriptor(),
            disposition: RegistrationDispositionV1::OutcomeUnknown,
            receipt_valid_until: 150,
        };

        let result = reconcile_historical_registration_attempt(
            ExpectedHistoricalRegistrationReconcilerProfileV1::from_trusted_configuration(
                foreign_descriptor(),
            ),
            &store,
            manifest(),
        );

        assert_eq!(
            result.disposition(),
            HistoricalReconciliationDispositionV1::OutcomeUnknown {
                reason: HistoricalReconciliationAmbiguityReasonV1::HistoricalStoreIdentityMismatch
            }
        );
        assert!(!result.permits_new_attempt());
    }

    #[test]
    fn historical_reconciliation_never_refreshes_expired_authority() {
        let store = HistoricalStore {
            descriptor: rotated_descriptor(),
            disposition: RegistrationDispositionV1::Persisted {
                record: RegistrationRecordCommitment::from_bytes(bytes(4)),
                post_frontier: frontier(8, Some(4)),
            },
            receipt_valid_until: 180,
        };

        let result = reconcile_historical_registration_attempt(
            ExpectedHistoricalRegistrationReconcilerProfileV1::from_trusted_configuration(
                rotated_descriptor(),
            ),
            &store,
            manifest(),
        );

        assert_eq!(result.reconciliation_valid_until(), Some(180));
        assert_eq!(result.authority_eligibility_valid_until(), Some(90));
    }

    #[test]
    fn final_absence_after_rotation_can_close_old_attempt() {
        let absence = RegistrationAttemptAbsenceCommitment::from_bytes(bytes(77));
        let store = HistoricalStore {
            descriptor: rotated_descriptor(),
            disposition: RegistrationDispositionV1::ProvenNotPersisted {
                observed_frontier: frontier(12, Some(9)),
                absence_evidence: absence,
            },
            receipt_valid_until: 150,
        };

        let result = reconcile_historical_registration_attempt(
            ExpectedHistoricalRegistrationReconcilerProfileV1::from_trusted_configuration(
                rotated_descriptor(),
            ),
            &store,
            manifest(),
        );

        assert!(result.permits_new_attempt());
        assert!(!result.persisted_candidate_may_be_rebound());
        assert!(!result.contains_effect_authority());
    }

    #[test]
    fn attempt_conflict_remains_unknown_after_rotation() {
        let store = HistoricalStore {
            descriptor: rotated_descriptor(),
            disposition: RegistrationDispositionV1::AttemptIdConflict {
                existing_manifest: manifest(),
            },
            receipt_valid_until: 150,
        };

        let result = reconcile_historical_registration_attempt(
            ExpectedHistoricalRegistrationReconcilerProfileV1::from_trusted_configuration(
                rotated_descriptor(),
            ),
            &store,
            manifest(),
        );

        assert_eq!(
            result.disposition(),
            HistoricalReconciliationDispositionV1::OutcomeUnknown {
                reason: HistoricalReconciliationAmbiguityReasonV1::AttemptIdConflict
            }
        );
        assert!(!result.permits_new_attempt());
    }
}

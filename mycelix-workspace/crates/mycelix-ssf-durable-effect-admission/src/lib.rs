// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Crash-safe durable effect admission for one exact SSF execution-surface
//! admission candidate.
//!
//! This crate records a write-ahead "possible effect" frontier. It does not
//! issue a move-only execution capability and never invokes an actuator.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_execution_surface_admission::{
    ExecutionSurfaceAdmissionCandidateV1, ExecutionSurfaceAdmissionReceiptV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

digest_type!(EffectAdmissionAttemptId);
digest_type!(EffectAdmissionRecordCommitment);
digest_type!(EffectAdmissionAbsenceCommitment);
digest_type!(EffectAdmissionReceiptCommitment);
digest_type!(EffectAdmissionStoreIdentityCommitment);
digest_type!(EffectAdmissionStorePolicyCommitment);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EffectAdmissionStoreDescriptorV1 {
    pub stable_identity: EffectAdmissionStoreIdentityCommitment,
    pub policy: EffectAdmissionStorePolicyCommitment,
    pub generation: u64,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DurableEffectAdmissionFrontierV1 {
    pub generation: u64,
    pub head: Option<EffectAdmissionRecordCommitment>,
}

/// Plain exact subject that is durably admitted before any effect capability
/// may be minted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectAdmissionSubjectV1 {
    pub execution_admission_receipt: ExecutionSurfaceAdmissionReceiptV1,
    pub execution_admission_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectAdmissionAttemptManifestV1 {
    pub schema_version: u16,
    pub attempt_id: EffectAdmissionAttemptId,
    pub subject: EffectAdmissionSubjectV1,
    pub expected_store: EffectAdmissionStoreDescriptorV1,
    pub expected_frontier: DurableEffectAdmissionFrontierV1,
}

impl EffectAdmissionAttemptManifestV1 {
    pub const fn new(
        attempt_id: EffectAdmissionAttemptId,
        subject: EffectAdmissionSubjectV1,
        expected_store: EffectAdmissionStoreDescriptorV1,
        expected_frontier: DurableEffectAdmissionFrontierV1,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            attempt_id,
            subject,
            expected_store,
            expected_frontier,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectAdmissionDispositionV1 {
    Admitted {
        record: EffectAdmissionRecordCommitment,
        post_frontier: DurableEffectAdmissionFrontierV1,
    },
    /// Final absence for this attempt ID. Once reported, this attempt may never
    /// later commit.
    ProvenNotAdmitted {
        absence: EffectAdmissionAbsenceCommitment,
        observed_frontier: DurableEffectAdmissionFrontierV1,
    },
    /// Same attempt ID was already bound to a different exact manifest.
    AttemptIdConflict,
    /// This exact execution-admission subject was already consumed by another
    /// durable effect-admission lineage.
    AdmissionAlreadyExists,
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectAdmissionReceiptV1 {
    pub schema_version: u16,
    pub store: EffectAdmissionStoreDescriptorV1,
    pub manifest: EffectAdmissionAttemptManifestV1,
    pub disposition: EffectAdmissionDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: EffectAdmissionReceiptCommitment,
}

pub trait EffectAdmissionStoreV1 {
    type Error;

    fn descriptor(&self) -> EffectAdmissionStoreDescriptorV1;

    /// External write boundary. The implementation contract requires:
    ///
    /// - first-seen attempt ID permanently binds the exact manifest;
    /// - one exact `EffectAdmissionSubjectV1` may produce at most one admitted
    ///   record across all attempt IDs;
    /// - `ProvenNotAdmitted` is final for that attempt ID;
    /// - `AdmissionAlreadyExists` never creates another record.
    fn admit_effect(
        &self,
        manifest: &EffectAdmissionAttemptManifestV1,
    ) -> Result<EffectAdmissionReceiptV1, Self::Error>;

    /// Read-only same-attempt reconciliation. This method must never create an
    /// admission record.
    fn reconcile_effect_admission(
        &self,
        manifest: &EffectAdmissionAttemptManifestV1,
    ) -> Result<EffectAdmissionReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedEffectAdmissionStoreProfileV1 {
    descriptor: EffectAdmissionStoreDescriptorV1,
}

impl ExpectedEffectAdmissionStoreProfileV1 {
    pub const fn from_trusted_configuration(descriptor: EffectAdmissionStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> EffectAdmissionStoreDescriptorV1 {
        self.descriptor
    }
}

// Sealed view: only the exact parent SSF admission-candidate typestate can enter
// this persistence boundary. External crates cannot implement the private
// supertrait for alternate bypass types.
mod sealed {
    pub trait Sealed {}
}

pub trait ExactExecutionAdmissionCandidateV1: sealed::Sealed {
    fn exact_admission_receipt(&self) -> ExecutionSurfaceAdmissionReceiptV1;
    fn exact_admission_valid_until(&self) -> u64;
}

impl<
        Q,
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
        UCQ,
        UBQ,
        UTQ,
        const L: usize,
        const R: usize,
    > sealed::Sealed
    for ExecutionSurfaceAdmissionCandidateV1<
        Q, S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >
{
}

impl<
        Q,
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
        UCQ,
        UBQ,
        UTQ,
        const L: usize,
        const R: usize,
    > ExactExecutionAdmissionCandidateV1
    for ExecutionSurfaceAdmissionCandidateV1<
        Q, S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >
{
    fn exact_admission_receipt(&self) -> ExecutionSurfaceAdmissionReceiptV1 {
        self.receipt()
    }

    fn exact_admission_valid_until(&self) -> u64 {
        self.valid_until()
    }
}

fn subject_from_candidate<C: ExactExecutionAdmissionCandidateV1>(
    candidate: &C,
) -> EffectAdmissionSubjectV1 {
    EffectAdmissionSubjectV1 {
        execution_admission_receipt: candidate.exact_admission_receipt(),
        execution_admission_valid_until: candidate.exact_admission_valid_until(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectAdmissionPreparationError {
    StoreDescriptorMismatchBefore,
}

pub struct EffectAdmissionPreparationFailureV1<C> {
    pub error: EffectAdmissionPreparationError,
    pub candidate: C,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectAdmissionUnknownReasonV1 {
    StoreErrorAfterWriteBoundary,
    StoreDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    ReceiptOutlivesAdmissionCandidate,
    InvalidAdmittedFrontier,
    InvalidFinalAbsenceFrontier,
    AttemptIdConflict,
    AdmissionAlreadyExists,
    StoreReportedUnknown,
}

pub struct FrozenEffectAdmissionV1<S, C> {
    candidate: C,
    expected: ExpectedEffectAdmissionStoreProfileV1,
    manifest: EffectAdmissionAttemptManifestV1,
    reason: EffectAdmissionUnknownReasonV1,
    _store: PhantomData<S>,
}

impl<S, C> FrozenEffectAdmissionV1<S, C> {
    pub const fn manifest(&self) -> EffectAdmissionAttemptManifestV1 {
        self.manifest
    }

    pub const fn reason(&self) -> EffectAdmissionUnknownReasonV1 {
        self.reason
    }

    pub const fn may_create_fresh_attempt(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub struct RecoverableEffectAdmissionCandidateV1<C> {
    candidate: C,
    manifest: EffectAdmissionAttemptManifestV1,
    receipt: EffectAdmissionReceiptV1,
}

impl<C> RecoverableEffectAdmissionCandidateV1<C> {
    pub const fn manifest(&self) -> EffectAdmissionAttemptManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> EffectAdmissionReceiptV1 {
        self.receipt
    }

    pub fn into_candidate(self) -> C {
        self.candidate
    }

    pub const fn may_create_fresh_attempt(&self) -> bool {
        true
    }
}

pub struct DurablyAdmittedEffectV1<S, C> {
    candidate: C,
    manifest: EffectAdmissionAttemptManifestV1,
    receipt: EffectAdmissionReceiptV1,
    admission_valid_until: u64,
    _store: PhantomData<S>,
}

impl<S, C> DurablyAdmittedEffectV1<S, C> {
    pub const fn manifest(&self) -> EffectAdmissionAttemptManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> EffectAdmissionReceiptV1 {
        self.receipt
    }

    pub const fn admission_valid_until(&self) -> u64 {
        self.admission_valid_until
    }

    pub const fn possible_effect_frontier_recorded(&self) -> bool {
        true
    }

    pub const fn requires_single_use_execution_capability(&self) -> bool {
        true
    }

    pub const fn contains_single_use_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub fn candidate(&self) -> &C {
        &self.candidate
    }
}

pub enum EffectAdmissionOutcomeV1<S, C> {
    Admitted(DurablyAdmittedEffectV1<S, C>),
    ProvenNotAdmitted(RecoverableEffectAdmissionCandidateV1<C>),
    OutcomeUnknown(FrozenEffectAdmissionV1<S, C>),
}

fn admitted_frontier_is_exact(
    expected: DurableEffectAdmissionFrontierV1,
    record: EffectAdmissionRecordCommitment,
    post: DurableEffectAdmissionFrontierV1,
) -> bool {
    expected
        .generation
        .checked_add(1)
        .is_some_and(|next| post.generation == next)
        && post.head == Some(record)
}

fn final_absence_frontier_is_compatible(
    expected: DurableEffectAdmissionFrontierV1,
    observed: DurableEffectAdmissionFrontierV1,
) -> bool {
    if observed.generation < expected.generation {
        return false;
    }
    if observed.generation == expected.generation {
        return observed.head == expected.head;
    }
    true
}

fn freeze<S, C>(
    candidate: C,
    expected: ExpectedEffectAdmissionStoreProfileV1,
    manifest: EffectAdmissionAttemptManifestV1,
    reason: EffectAdmissionUnknownReasonV1,
) -> EffectAdmissionOutcomeV1<S, C> {
    EffectAdmissionOutcomeV1::OutcomeUnknown(FrozenEffectAdmissionV1 {
        candidate,
        expected,
        manifest,
        reason,
        _store: PhantomData,
    })
}

fn interpret_receipt<S, C>(
    candidate: C,
    expected: ExpectedEffectAdmissionStoreProfileV1,
    manifest: EffectAdmissionAttemptManifestV1,
    receipt: EffectAdmissionReceiptV1,
) -> EffectAdmissionOutcomeV1<S, C>
where
    C: ExactExecutionAdmissionCandidateV1,
{
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::UnsupportedReceiptSchema,
        );
    }
    if receipt.store != expected.descriptor {
        return freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::ReceiptStoreMismatch,
        );
    }
    if receipt.manifest != manifest {
        return freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::ReceiptManifestMismatch,
        );
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::ReceiptOutlivesStore,
        );
    }
    if receipt.valid_until > candidate.exact_admission_valid_until() {
        return freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::ReceiptOutlivesAdmissionCandidate,
        );
    }

    match receipt.disposition {
        EffectAdmissionDispositionV1::Admitted {
            record,
            post_frontier,
        } => {
            if !admitted_frontier_is_exact(manifest.expected_frontier, record, post_frontier) {
                return freeze::<S, C>(
                    candidate,
                    expected,
                    manifest,
                    EffectAdmissionUnknownReasonV1::InvalidAdmittedFrontier,
                );
            }
            let admission_valid_until = receipt
                .valid_until
                .min(candidate.exact_admission_valid_until());
            EffectAdmissionOutcomeV1::Admitted(DurablyAdmittedEffectV1 {
                candidate,
                manifest,
                receipt,
                admission_valid_until,
                _store: PhantomData,
            })
        }
        EffectAdmissionDispositionV1::ProvenNotAdmitted {
            observed_frontier, ..
        } => {
            if !final_absence_frontier_is_compatible(
                manifest.expected_frontier,
                observed_frontier,
            ) {
                return freeze::<S, C>(
                    candidate,
                    expected,
                    manifest,
                    EffectAdmissionUnknownReasonV1::InvalidFinalAbsenceFrontier,
                );
            }
            EffectAdmissionOutcomeV1::ProvenNotAdmitted(RecoverableEffectAdmissionCandidateV1 {
                candidate,
                manifest,
                receipt,
            })
        }
        EffectAdmissionDispositionV1::AttemptIdConflict => freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::AttemptIdConflict,
        ),
        EffectAdmissionDispositionV1::AdmissionAlreadyExists => freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::AdmissionAlreadyExists,
        ),
        EffectAdmissionDispositionV1::OutcomeUnknown => freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::StoreReportedUnknown,
        ),
    }
}

pub fn admit_effect<S, C>(
    expected: ExpectedEffectAdmissionStoreProfileV1,
    store: &S,
    candidate: C,
    attempt_id: EffectAdmissionAttemptId,
    expected_frontier: DurableEffectAdmissionFrontierV1,
) -> Result<EffectAdmissionOutcomeV1<S, C>, EffectAdmissionPreparationFailureV1<C>>
where
    S: EffectAdmissionStoreV1,
    C: ExactExecutionAdmissionCandidateV1,
{
    if store.descriptor() != expected.descriptor {
        return Err(EffectAdmissionPreparationFailureV1 {
            error: EffectAdmissionPreparationError::StoreDescriptorMismatchBefore,
            candidate,
        });
    }

    let manifest = EffectAdmissionAttemptManifestV1::new(
        attempt_id,
        subject_from_candidate(&candidate),
        expected.descriptor,
        expected_frontier,
    );

    let receipt = match store.admit_effect(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Ok(freeze::<S, C>(
                candidate,
                expected,
                manifest,
                EffectAdmissionUnknownReasonV1::StoreErrorAfterWriteBoundary,
            ));
        }
    };

    if store.descriptor() != expected.descriptor {
        return Ok(freeze::<S, C>(
            candidate,
            expected,
            manifest,
            EffectAdmissionUnknownReasonV1::StoreDescriptorMismatchAfter,
        ));
    }

    Ok(interpret_receipt::<S, C>(
        candidate, candidate_expected(expected), manifest, receipt,
    ))
}

const fn candidate_expected(
    expected: ExpectedEffectAdmissionStoreProfileV1,
) -> ExpectedEffectAdmissionStoreProfileV1 {
    expected
}

impl<S, C> FrozenEffectAdmissionV1<S, C>
where
    S: EffectAdmissionStoreV1,
    C: ExactExecutionAdmissionCandidateV1,
{
    /// Reconcile only the same exact attempt. No new write is permitted.
    pub fn reconcile(self, store: &S) -> EffectAdmissionOutcomeV1<S, C> {
        if store.descriptor() != self.expected.descriptor {
            return EffectAdmissionOutcomeV1::OutcomeUnknown(self);
        }
        let receipt = match store.reconcile_effect_admission(&self.manifest) {
            Ok(receipt) => receipt,
            Err(_) => return EffectAdmissionOutcomeV1::OutcomeUnknown(self),
        };
        if store.descriptor() != self.expected.descriptor {
            return EffectAdmissionOutcomeV1::OutcomeUnknown(self);
        }
        interpret_receipt::<S, C>(self.candidate, self.expected, self.manifest, receipt)
    }
}

/// Restart-safe read-only reconciliation from a journaled manifest alone.
///
/// This can establish historical disposition evidence but cannot recreate the
/// typed execution-admission candidate or an effect capability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoricalEffectAdmissionEvidenceV1 {
    pub manifest: EffectAdmissionAttemptManifestV1,
    pub receipt: EffectAdmissionReceiptV1,
    pub authority_eligibility_valid_until: Option<u64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalEffectAdmissionError {
    StoreDescriptorMismatchBefore,
    StoreQueryFailed,
    StoreDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    InvalidAdmittedFrontier,
    InvalidFinalAbsenceFrontier,
}

pub fn reconcile_effect_admission_manifest<S: EffectAdmissionStoreV1>(
    expected: ExpectedEffectAdmissionStoreProfileV1,
    store: &S,
    manifest: EffectAdmissionAttemptManifestV1,
) -> Result<HistoricalEffectAdmissionEvidenceV1, HistoricalEffectAdmissionError> {
    if store.descriptor() != expected.descriptor {
        return Err(HistoricalEffectAdmissionError::StoreDescriptorMismatchBefore);
    }
    let receipt = store
        .reconcile_effect_admission(&manifest)
        .map_err(|_| HistoricalEffectAdmissionError::StoreQueryFailed)?;
    if store.descriptor() != expected.descriptor {
        return Err(HistoricalEffectAdmissionError::StoreDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalEffectAdmissionError::UnsupportedReceiptSchema);
    }
    if receipt.store != expected.descriptor {
        return Err(HistoricalEffectAdmissionError::ReceiptStoreMismatch);
    }
    if receipt.manifest != manifest {
        return Err(HistoricalEffectAdmissionError::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return Err(HistoricalEffectAdmissionError::ReceiptOutlivesStore);
    }

    let authority_eligibility_valid_until = match receipt.disposition {
        EffectAdmissionDispositionV1::Admitted {
            record,
            post_frontier,
        } => {
            if !admitted_frontier_is_exact(manifest.expected_frontier, record, post_frontier) {
                return Err(HistoricalEffectAdmissionError::InvalidAdmittedFrontier);
            }
            Some(receipt.valid_until.min(manifest.subject.execution_admission_valid_until))
        }
        EffectAdmissionDispositionV1::ProvenNotAdmitted {
            observed_frontier, ..
        } => {
            if !final_absence_frontier_is_compatible(
                manifest.expected_frontier,
                observed_frontier,
            ) {
                return Err(HistoricalEffectAdmissionError::InvalidFinalAbsenceFrontier);
            }
            None
        }
        EffectAdmissionDispositionV1::AttemptIdConflict
        | EffectAdmissionDispositionV1::AdmissionAlreadyExists
        | EffectAdmissionDispositionV1::OutcomeUnknown => None,
    };

    Ok(HistoricalEffectAdmissionEvidenceV1 {
        manifest,
        receipt,
        authority_eligibility_valid_until,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    #[test]
    fn admitted_frontier_advances_exactly_once_to_record() {
        let expected = DurableEffectAdmissionFrontierV1 {
            generation: 7,
            head: Some(EffectAdmissionRecordCommitment::from_bytes(bytes(1))),
        };
        let record = EffectAdmissionRecordCommitment::from_bytes(bytes(2));
        assert!(admitted_frontier_is_exact(
            expected,
            record,
            DurableEffectAdmissionFrontierV1 {
                generation: 8,
                head: Some(record),
            }
        ));
        assert!(!admitted_frontier_is_exact(
            expected,
            record,
            DurableEffectAdmissionFrontierV1 {
                generation: 9,
                head: Some(record),
            }
        ));
    }

    #[test]
    fn final_nonadmission_never_rolls_frontier_back() {
        let expected = DurableEffectAdmissionFrontierV1 {
            generation: 7,
            head: Some(EffectAdmissionRecordCommitment::from_bytes(bytes(1))),
        };
        assert!(!final_absence_frontier_is_compatible(
            expected,
            DurableEffectAdmissionFrontierV1 {
                generation: 6,
                head: None,
            }
        ));
        assert!(final_absence_frontier_is_compatible(
            expected,
            DurableEffectAdmissionFrontierV1 {
                generation: 8,
                head: Some(EffectAdmissionRecordCommitment::from_bytes(bytes(9))),
            }
        ));
    }

    #[test]
    fn same_generation_absence_requires_exact_head() {
        let expected = DurableEffectAdmissionFrontierV1 {
            generation: 7,
            head: Some(EffectAdmissionRecordCommitment::from_bytes(bytes(1))),
        };
        assert!(!final_absence_frontier_is_compatible(
            expected,
            DurableEffectAdmissionFrontierV1 {
                generation: 7,
                head: Some(EffectAdmissionRecordCommitment::from_bytes(bytes(2))),
            }
        ));
    }

    #[test]
    fn already_existing_admission_is_not_final_absence() {
        assert_ne!(
            EffectAdmissionDispositionV1::AdmissionAlreadyExists,
            EffectAdmissionDispositionV1::ProvenNotAdmitted {
                absence: EffectAdmissionAbsenceCommitment::from_bytes(bytes(3)),
                observed_frontier: DurableEffectAdmissionFrontierV1 {
                    generation: 4,
                    head: None,
                },
            }
        );
    }
}

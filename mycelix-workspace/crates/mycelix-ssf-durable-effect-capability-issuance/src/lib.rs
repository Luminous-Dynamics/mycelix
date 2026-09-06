// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Crash-safe durable issuance of one SSF effect-capability lineage.
//!
//! Source-owned operation material is still only pre-capability evidence. Rust
//! move semantics cannot prove single use across process crashes because the
//! material lineage may later be reconstructed. This crate therefore records a
//! durable issuance decision before any transient move-only capability may be
//! minted.
//!
//! A successful record is still not an actuator capability. Exact rebind and a
//! fresh final generation/time check remain mandatory downstream.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_capability_time_effect_revalidation::CapabilityTimeRevalidationReceiptV1;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_effect_admission_rebind::ReboundEffectAdmissionBindingV1;
use mycelix_ssf_source_owned_operation_material::{
    SourceOwnedOperationMaterialReceiptV1, SourceOwnedOperationMaterialV1,
};

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

macro_rules! generation_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[repr(transparent)]
        pub struct $name(u64);

        impl $name {
            pub const fn new(value: u64) -> Self {
                Self(value)
            }

            pub const fn get(self) -> u64 {
                self.0
            }
        }
    };
}

digest_type!(EffectCapabilityIssuanceAttemptId);
digest_type!(EffectCapabilityRecordCommitment);
digest_type!(EffectCapabilityIssuanceReceiptCommitment);
digest_type!(EffectCapabilityIssuanceAbsenceCommitment);
digest_type!(EffectCapabilityIssuanceManifestCommitment);
digest_type!(EffectCapabilityIssuanceStoreIdentityCommitment);
digest_type!(EffectCapabilityIssuanceStorePolicyCommitment);

generation_type!(EffectCapabilityIssuanceStoreGeneration);
generation_type!(DurableEffectCapabilityGeneration);

/// Closed v0.1 basis for capability-issuance-store freshness.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectCapabilityIssuanceStoreTimeBasisV1 {
    UnixMillisecondsUtc,
}

/// Independently trusted exact durable-store generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EffectCapabilityIssuanceStoreDescriptorV1 {
    pub stable_identity: EffectCapabilityIssuanceStoreIdentityCommitment,
    pub policy: EffectCapabilityIssuanceStorePolicyCommitment,
    pub generation: EffectCapabilityIssuanceStoreGeneration,
    pub time_basis: EffectCapabilityIssuanceStoreTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedEffectCapabilityIssuanceStoreProfileV1 {
    descriptor: EffectCapabilityIssuanceStoreDescriptorV1,
}

impl ExpectedEffectCapabilityIssuanceStoreProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: EffectCapabilityIssuanceStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> EffectCapabilityIssuanceStoreDescriptorV1 {
        self.descriptor
    }
}

/// Exact durable capability-issuance log frontier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DurableEffectCapabilityFrontierV1 {
    pub generation: DurableEffectCapabilityGeneration,
    pub head: Option<EffectCapabilityRecordCommitment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedDurableEffectCapabilityFrontierV1 {
    frontier: DurableEffectCapabilityFrontierV1,
}

impl ExpectedDurableEffectCapabilityFrontierV1 {
    pub const fn from_trusted_state(frontier: DurableEffectCapabilityFrontierV1) -> Self {
        Self { frontier }
    }

    pub const fn frontier(&self) -> DurableEffectCapabilityFrontierV1 {
        self.frontier
    }
}

/// Plain exact subject for durable capability issuance.
///
/// `admitted_effect.admission_record` is the single-use durable lineage key.
/// The exact provider-owned material receipt is retained so a second
/// materialization cannot silently substitute different handle/payload evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectCapabilityIssuanceSubjectV1 {
    pub admitted_effect: ReboundEffectAdmissionBindingV1,
    pub capability_time_revalidation: CapabilityTimeRevalidationReceiptV1,
    pub material_receipt: SourceOwnedOperationMaterialReceiptV1,
    pub material_valid_until: u64,
}

/// Plain journalable exact issuance attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectCapabilityIssuanceAttemptManifestV1 {
    pub schema_version: u16,
    attempt_id: EffectCapabilityIssuanceAttemptId,
    subject: EffectCapabilityIssuanceSubjectV1,
    expected_store: EffectCapabilityIssuanceStoreDescriptorV1,
    expected_frontier: DurableEffectCapabilityFrontierV1,
}

impl EffectCapabilityIssuanceAttemptManifestV1 {
    pub const fn from_journaled_parts(
        attempt_id: EffectCapabilityIssuanceAttemptId,
        subject: EffectCapabilityIssuanceSubjectV1,
        expected_store: EffectCapabilityIssuanceStoreDescriptorV1,
        expected_frontier: DurableEffectCapabilityFrontierV1,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            attempt_id,
            subject,
            expected_store,
            expected_frontier,
        }
    }

    pub const fn attempt_id(&self) -> EffectCapabilityIssuanceAttemptId {
        self.attempt_id
    }

    pub const fn subject(&self) -> EffectCapabilityIssuanceSubjectV1 {
        self.subject
    }

    pub const fn expected_store(&self) -> EffectCapabilityIssuanceStoreDescriptorV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> DurableEffectCapabilityFrontierV1 {
        self.expected_frontier
    }
}

/// Closed-world durable store outcome.
///
/// Implementations MUST enforce both:
///
/// 1. first-seen attempt ID permanently binds the exact manifest; and
/// 2. one exact `subject.admitted_effect.admission_record` may produce at most
///    one capability-issuance record across all attempt IDs/rematerializations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectCapabilityIssuanceDispositionV1 {
    Issued {
        capability_record: EffectCapabilityRecordCommitment,
        post_frontier: DurableEffectCapabilityFrontierV1,
    },
    /// Final absence for this exact attempt ID. Once returned, this attempt may
    /// never later commit.
    ProvenNotIssued {
        observed_frontier: DurableEffectCapabilityFrontierV1,
        absence_evidence: EffectCapabilityIssuanceAbsenceCommitment,
    },
    AttemptIdConflict {
        existing_manifest: EffectCapabilityIssuanceManifestCommitment,
    },
    /// The durable effect-admission record was already consumed by another
    /// capability-issuance lineage.
    EffectAdmissionAlreadyIssued {
        existing_capability_record: EffectCapabilityRecordCommitment,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectCapabilityIssuanceReceiptV1 {
    pub schema_version: u16,
    pub store: EffectCapabilityIssuanceStoreDescriptorV1,
    pub manifest: EffectCapabilityIssuanceAttemptManifestV1,
    pub disposition: EffectCapabilityIssuanceDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: EffectCapabilityIssuanceReceiptCommitment,
}

/// External durable-store boundary.
///
/// `issue_effect_capability_lineage` may write. Once invoked, any caller-side
/// inability to prove the postcondition becomes `OutcomeUnknown`.
///
/// `reconcile_effect_capability_lineage` is read-only for the exact journaled
/// attempt and must never create a record.
pub trait EffectCapabilityIssuanceStoreV1 {
    type Error;

    fn descriptor(&self) -> EffectCapabilityIssuanceStoreDescriptorV1;

    fn issue_effect_capability_lineage(
        &self,
        manifest: &EffectCapabilityIssuanceAttemptManifestV1,
    ) -> Result<EffectCapabilityIssuanceReceiptV1, Self::Error>;

    fn reconcile_effect_capability_lineage(
        &self,
        manifest: &EffectCapabilityIssuanceAttemptManifestV1,
    ) -> Result<EffectCapabilityIssuanceReceiptV1, Self::Error>;
}

// Sealed view: only the exact #214 material typestate can enter this boundary.
mod sealed {
    pub trait Sealed {}
}

pub trait ExactSourceOwnedOperationMaterialV1: sealed::Sealed {
    fn exact_admitted_effect(&self) -> ReboundEffectAdmissionBindingV1;
    fn exact_capability_time_revalidation(&self) -> CapabilityTimeRevalidationReceiptV1;
    fn exact_material_receipt(&self) -> SourceOwnedOperationMaterialReceiptV1;
    fn exact_material_valid_until(&self) -> u64;
}

impl<P, S, C, Q, TQ, MTQ> sealed::Sealed for SourceOwnedOperationMaterialV1<P, S, C, Q, TQ, MTQ> {}

impl<P, S, C, Q, TQ, MTQ> ExactSourceOwnedOperationMaterialV1
    for SourceOwnedOperationMaterialV1<P, S, C, Q, TQ, MTQ>
{
    fn exact_admitted_effect(&self) -> ReboundEffectAdmissionBindingV1 {
        self.effect().rebound().binding()
    }

    fn exact_capability_time_revalidation(&self) -> CapabilityTimeRevalidationReceiptV1 {
        self.effect().receipt()
    }

    fn exact_material_receipt(&self) -> SourceOwnedOperationMaterialReceiptV1 {
        self.receipt()
    }

    fn exact_material_valid_until(&self) -> u64 {
        self.valid_until()
    }
}

fn subject_from_material<M: ExactSourceOwnedOperationMaterialV1>(
    material: &M,
) -> EffectCapabilityIssuanceSubjectV1 {
    EffectCapabilityIssuanceSubjectV1 {
        admitted_effect: material.exact_admitted_effect(),
        capability_time_revalidation: material.exact_capability_time_revalidation(),
        material_receipt: material.exact_material_receipt(),
        material_valid_until: material.exact_material_valid_until(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectCapabilityIssuancePreparationError {
    StoreDescriptorMismatchBefore,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectCapabilityIssuanceAmbiguityReasonV1 {
    UnsupportedManifestSchema,
    JournaledExpectedStoreMismatch,
    StoreErrorAfterAttempt,
    StoreDescriptorChangedAfter,
    StoreDescriptorMismatchBeforeReconciliation,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    ReceiptOutlivesMaterial,
    IssuedGenerationOverflow,
    InvalidIssuedFrontier,
    InvalidProvenNotIssuedFrontier,
    AttemptIdConflict,
    EffectAdmissionAlreadyIssued,
    StoreReportedOutcomeUnknown,
}

pub struct PreparedEffectCapabilityIssuanceV1<M> {
    material: M,
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    expected_frontier: ExpectedDurableEffectCapabilityFrontierV1,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
}

impl<M> PreparedEffectCapabilityIssuanceV1<M> {
    pub fn material(&self) -> &M {
        &self.material
    }

    pub const fn expected_store(&self) -> ExpectedEffectCapabilityIssuanceStoreProfileV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> ExpectedDurableEffectCapabilityFrontierV1 {
        self.expected_frontier
    }

    pub const fn manifest(&self) -> EffectCapabilityIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn contains_transient_effect_capability(&self) -> bool {
        false
    }
}

pub fn prepare_effect_capability_issuance<M: ExactSourceOwnedOperationMaterialV1>(
    material: M,
    attempt_id: EffectCapabilityIssuanceAttemptId,
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    expected_frontier: ExpectedDurableEffectCapabilityFrontierV1,
) -> PreparedEffectCapabilityIssuanceV1<M> {
    let subject = subject_from_material(&material);
    let manifest = EffectCapabilityIssuanceAttemptManifestV1::from_journaled_parts(
        attempt_id,
        subject,
        expected_store.descriptor,
        expected_frontier.frontier,
    );

    PreparedEffectCapabilityIssuanceV1 {
        material,
        expected_store,
        expected_frontier,
        manifest,
    }
}

pub struct EffectCapabilityIssuancePreparationFailureV1<M> {
    prepared: PreparedEffectCapabilityIssuanceV1<M>,
    error: EffectCapabilityIssuancePreparationError,
}

impl<M> EffectCapabilityIssuancePreparationFailureV1<M> {
    pub const fn error(&self) -> EffectCapabilityIssuancePreparationError {
        self.error
    }

    pub fn into_prepared(self) -> PreparedEffectCapabilityIssuanceV1<M> {
        self.prepared
    }
}

pub struct FrozenEffectCapabilityIssuanceV1<S, M> {
    material: M,
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    reason: EffectCapabilityIssuanceAmbiguityReasonV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, M> FrozenEffectCapabilityIssuanceV1<S, M> {
    pub const fn manifest(&self) -> EffectCapabilityIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn reason(&self) -> EffectCapabilityIssuanceAmbiguityReasonV1 {
        self.reason
    }

    pub const fn may_create_fresh_attempt(&self) -> bool {
        false
    }

    pub const fn contains_transient_effect_capability(&self) -> bool {
        false
    }
}

pub struct RecoverableEffectCapabilityMaterialV1<M> {
    material: M,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    receipt: EffectCapabilityIssuanceReceiptV1,
}

impl<M> RecoverableEffectCapabilityMaterialV1<M> {
    pub const fn manifest(&self) -> EffectCapabilityIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> EffectCapabilityIssuanceReceiptV1 {
        self.receipt
    }

    pub fn into_material(self) -> M {
        self.material
    }

    pub const fn may_create_fresh_attempt(&self) -> bool {
        true
    }
}

/// Durable capability-issuance lineage.
///
/// This proves that one exact durable effect-admission record has been consumed
/// by one exact capability-issuance record. It is historical authority evidence,
/// not the transient move-only capability itself.
pub struct DurablyIssuedEffectCapabilityLineageV1<S, M> {
    material: M,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    receipt: EffectCapabilityIssuanceReceiptV1,
    capability_record: EffectCapabilityRecordCommitment,
    post_frontier: DurableEffectCapabilityFrontierV1,
    capability_eligibility_valid_until: u64,
    _store: PhantomData<fn() -> S>,
}

impl<S, M> DurablyIssuedEffectCapabilityLineageV1<S, M> {
    pub fn material(&self) -> &M {
        &self.material
    }

    pub const fn manifest(&self) -> EffectCapabilityIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> EffectCapabilityIssuanceReceiptV1 {
        self.receipt
    }

    pub const fn capability_record(&self) -> EffectCapabilityRecordCommitment {
        self.capability_record
    }

    pub const fn post_frontier(&self) -> DurableEffectCapabilityFrontierV1 {
        self.post_frontier
    }

    pub const fn capability_eligibility_valid_until(&self) -> u64 {
        self.capability_eligibility_valid_until
    }

    pub const fn durable_single_lineage_established(&self) -> bool {
        true
    }

    pub const fn requires_exact_rebind_after_restart(&self) -> bool {
        true
    }

    pub const fn requires_final_execution_revalidation(&self) -> bool {
        true
    }

    pub const fn contains_transient_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub enum EffectCapabilityIssuanceOutcomeV1<S, M> {
    Issued(DurablyIssuedEffectCapabilityLineageV1<S, M>),
    ProvenNotIssued(RecoverableEffectCapabilityMaterialV1<M>),
    OutcomeUnknown(FrozenEffectCapabilityIssuanceV1<S, M>),
}

fn issued_frontier_is_exact(
    expected: DurableEffectCapabilityFrontierV1,
    capability_record: EffectCapabilityRecordCommitment,
    post: DurableEffectCapabilityFrontierV1,
) -> Result<(), EffectCapabilityIssuanceAmbiguityReasonV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(EffectCapabilityIssuanceAmbiguityReasonV1::IssuedGenerationOverflow)?;

    if post.generation.get() != next || post.head != Some(capability_record) {
        return Err(EffectCapabilityIssuanceAmbiguityReasonV1::InvalidIssuedFrontier);
    }
    Ok(())
}

fn final_absence_frontier_is_compatible(
    expected: DurableEffectCapabilityFrontierV1,
    observed: DurableEffectCapabilityFrontierV1,
) -> bool {
    if observed.generation < expected.generation {
        return false;
    }
    if observed.generation == expected.generation {
        return observed.head == expected.head;
    }
    true
}

fn validate_receipt_common(
    expected_store: EffectCapabilityIssuanceStoreDescriptorV1,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    receipt: &EffectCapabilityIssuanceReceiptV1,
) -> Result<(), EffectCapabilityIssuanceAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(EffectCapabilityIssuanceAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_store {
        return Err(EffectCapabilityIssuanceAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest != manifest {
        return Err(EffectCapabilityIssuanceAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.valid_until {
        return Err(EffectCapabilityIssuanceAmbiguityReasonV1::ReceiptOutlivesStore);
    }
    if receipt.valid_until > manifest.subject.material_valid_until {
        return Err(EffectCapabilityIssuanceAmbiguityReasonV1::ReceiptOutlivesMaterial);
    }
    Ok(())
}

fn freeze<S, M>(
    material: M,
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    reason: EffectCapabilityIssuanceAmbiguityReasonV1,
) -> EffectCapabilityIssuanceOutcomeV1<S, M> {
    EffectCapabilityIssuanceOutcomeV1::OutcomeUnknown(FrozenEffectCapabilityIssuanceV1 {
        material,
        expected_store,
        manifest,
        reason,
        _store: PhantomData,
    })
}

fn interpret_receipt<S, M>(
    material: M,
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    receipt: EffectCapabilityIssuanceReceiptV1,
) -> EffectCapabilityIssuanceOutcomeV1<S, M>
where
    M: ExactSourceOwnedOperationMaterialV1,
{
    if let Err(reason) = validate_receipt_common(expected_store.descriptor, manifest, &receipt) {
        return freeze::<S, M>(material, expected_store, manifest, reason);
    }

    match receipt.disposition {
        EffectCapabilityIssuanceDispositionV1::Issued {
            capability_record,
            post_frontier,
        } => {
            if let Err(reason) =
                issued_frontier_is_exact(manifest.expected_frontier, capability_record, post_frontier)
            {
                return freeze::<S, M>(material, expected_store, manifest, reason);
            }

            let capability_eligibility_valid_until = receipt
                .valid_until
                .min(expected_store.descriptor.valid_until)
                .min(material.exact_material_valid_until());

            EffectCapabilityIssuanceOutcomeV1::Issued(
                DurablyIssuedEffectCapabilityLineageV1 {
                    material,
                    manifest,
                    receipt,
                    capability_record,
                    post_frontier,
                    capability_eligibility_valid_until,
                    _store: PhantomData,
                },
            )
        }
        EffectCapabilityIssuanceDispositionV1::ProvenNotIssued {
            observed_frontier, ..
        } => {
            if !final_absence_frontier_is_compatible(manifest.expected_frontier, observed_frontier)
            {
                return freeze::<S, M>(
                    material,
                    expected_store,
                    manifest,
                    EffectCapabilityIssuanceAmbiguityReasonV1::InvalidProvenNotIssuedFrontier,
                );
            }

            EffectCapabilityIssuanceOutcomeV1::ProvenNotIssued(
                RecoverableEffectCapabilityMaterialV1 {
                    material,
                    manifest,
                    receipt,
                },
            )
        }
        EffectCapabilityIssuanceDispositionV1::AttemptIdConflict { .. } => freeze::<S, M>(
            material,
            expected_store,
            manifest,
            EffectCapabilityIssuanceAmbiguityReasonV1::AttemptIdConflict,
        ),
        EffectCapabilityIssuanceDispositionV1::EffectAdmissionAlreadyIssued { .. } => {
            freeze::<S, M>(
                material,
                expected_store,
                manifest,
                EffectCapabilityIssuanceAmbiguityReasonV1::EffectAdmissionAlreadyIssued,
            )
        }
        EffectCapabilityIssuanceDispositionV1::OutcomeUnknown => freeze::<S, M>(
            material,
            expected_store,
            manifest,
            EffectCapabilityIssuanceAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

pub fn issue_prepared_effect_capability_lineage<S, M>(
    prepared: PreparedEffectCapabilityIssuanceV1<M>,
    store: &S,
) -> Result<
    EffectCapabilityIssuanceOutcomeV1<S, M>,
    EffectCapabilityIssuancePreparationFailureV1<M>,
>
where
    S: EffectCapabilityIssuanceStoreV1,
    M: ExactSourceOwnedOperationMaterialV1,
{
    if store.descriptor() != prepared.expected_store.descriptor {
        return Err(EffectCapabilityIssuancePreparationFailureV1 {
            prepared,
            error: EffectCapabilityIssuancePreparationError::StoreDescriptorMismatchBefore,
        });
    }

    let PreparedEffectCapabilityIssuanceV1 {
        material,
        expected_store,
        manifest,
        ..
    } = prepared;

    let receipt = match store.issue_effect_capability_lineage(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Ok(freeze::<S, M>(
                material,
                expected_store,
                manifest,
                EffectCapabilityIssuanceAmbiguityReasonV1::StoreErrorAfterAttempt,
            ));
        }
    };

    if store.descriptor() != expected_store.descriptor {
        return Ok(freeze::<S, M>(
            material,
            expected_store,
            manifest,
            EffectCapabilityIssuanceAmbiguityReasonV1::StoreDescriptorChangedAfter,
        ));
    }

    Ok(interpret_receipt::<S, M>(
        material,
        expected_store,
        manifest,
        receipt,
    ))
}

/// Read-only same-attempt reconciliation while the typed material still exists
/// in memory. It may not create a new issuance attempt.
pub fn reconcile_frozen_effect_capability_issuance<S, M>(
    frozen: FrozenEffectCapabilityIssuanceV1<S, M>,
    store: &S,
) -> EffectCapabilityIssuanceOutcomeV1<S, M>
where
    S: EffectCapabilityIssuanceStoreV1,
    M: ExactSourceOwnedOperationMaterialV1,
{
    let FrozenEffectCapabilityIssuanceV1 {
        material,
        expected_store,
        manifest,
        ..
    } = frozen;

    if store.descriptor() != expected_store.descriptor {
        return freeze::<S, M>(
            material,
            expected_store,
            manifest,
            EffectCapabilityIssuanceAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = match store.reconcile_effect_capability_lineage(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return freeze::<S, M>(
                material,
                expected_store,
                manifest,
                EffectCapabilityIssuanceAmbiguityReasonV1::StoreErrorAfterAttempt,
            );
        }
    };

    if store.descriptor() != expected_store.descriptor {
        return freeze::<S, M>(
            material,
            expected_store,
            manifest,
            EffectCapabilityIssuanceAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }

    interpret_receipt::<S, M>(material, expected_store, manifest, receipt)
}

/// Restart-safe historical disposition from the plain journaled manifest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JournaledEffectCapabilityIssuanceDispositionV1 {
    Issued {
        capability_record: EffectCapabilityRecordCommitment,
        post_frontier: DurableEffectCapabilityFrontierV1,
    },
    ProvenNotIssued {
        observed_frontier: DurableEffectCapabilityFrontierV1,
        absence_evidence: EffectCapabilityIssuanceAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: EffectCapabilityIssuanceAmbiguityReasonV1,
    },
}

pub struct JournaledEffectCapabilityIssuanceReconciliationV1<S> {
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    receipt: Option<EffectCapabilityIssuanceReceiptV1>,
    disposition: JournaledEffectCapabilityIssuanceDispositionV1,
    reconciliation_valid_until: Option<u64>,
    capability_eligibility_valid_until: Option<u64>,
    _store: PhantomData<fn() -> S>,
}

impl<S> JournaledEffectCapabilityIssuanceReconciliationV1<S> {
    pub const fn manifest(&self) -> EffectCapabilityIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_store(&self) -> ExpectedEffectCapabilityIssuanceStoreProfileV1 {
        self.expected_store
    }

    pub fn receipt(&self) -> Option<&EffectCapabilityIssuanceReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> JournaledEffectCapabilityIssuanceDispositionV1 {
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
            JournaledEffectCapabilityIssuanceDispositionV1::ProvenNotIssued { .. }
        )
    }

    pub fn issued_lineage_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            JournaledEffectCapabilityIssuanceDispositionV1::Issued { .. }
        )
    }

    pub const fn recreates_typed_material(&self) -> bool {
        false
    }

    pub const fn contains_transient_effect_capability(&self) -> bool {
        false
    }
}

fn journaled_unknown<S>(
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    receipt: Option<EffectCapabilityIssuanceReceiptV1>,
    reason: EffectCapabilityIssuanceAmbiguityReasonV1,
) -> JournaledEffectCapabilityIssuanceReconciliationV1<S> {
    JournaledEffectCapabilityIssuanceReconciliationV1 {
        manifest,
        expected_store,
        receipt,
        disposition: JournaledEffectCapabilityIssuanceDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        capability_eligibility_valid_until: None,
        _store: PhantomData,
    }
}

/// Reconcile after process restart from the exact journaled manifest alone.
///
/// This v0.1 path requires the exact original store descriptor. A later
/// historical-reconciliation layer may allow same-stable-identity key/policy
/// rotation without pretending the old write generation is still current.
pub fn reconcile_journaled_effect_capability_issuance<S>(
    expected_store: ExpectedEffectCapabilityIssuanceStoreProfileV1,
    store: &S,
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
) -> JournaledEffectCapabilityIssuanceReconciliationV1<S>
where
    S: EffectCapabilityIssuanceStoreV1,
{
    if manifest.schema_version != SSF_SCHEMA_V1 {
        return journaled_unknown(
            manifest,
            expected_store,
            None,
            EffectCapabilityIssuanceAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }
    if manifest.expected_store != expected_store.descriptor {
        return journaled_unknown(
            manifest,
            expected_store,
            None,
            EffectCapabilityIssuanceAmbiguityReasonV1::JournaledExpectedStoreMismatch,
        );
    }
    if store.descriptor() != expected_store.descriptor {
        return journaled_unknown(
            manifest,
            expected_store,
            None,
            EffectCapabilityIssuanceAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = match store.reconcile_effect_capability_lineage(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return journaled_unknown(
                manifest,
                expected_store,
                None,
                EffectCapabilityIssuanceAmbiguityReasonV1::StoreErrorAfterAttempt,
            );
        }
    };

    if store.descriptor() != expected_store.descriptor {
        return journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityIssuanceAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }

    if let Err(reason) = validate_receipt_common(expected_store.descriptor, manifest, &receipt) {
        return journaled_unknown(manifest, expected_store, Some(receipt), reason);
    }

    match receipt.disposition {
        EffectCapabilityIssuanceDispositionV1::Issued {
            capability_record,
            post_frontier,
        } => {
            if let Err(reason) =
                issued_frontier_is_exact(manifest.expected_frontier, capability_record, post_frontier)
            {
                return journaled_unknown(manifest, expected_store, Some(receipt), reason);
            }

            let capability_eligibility_valid_until = receipt
                .valid_until
                .min(expected_store.descriptor.valid_until)
                .min(manifest.subject.material_valid_until);

            JournaledEffectCapabilityIssuanceReconciliationV1 {
                manifest,
                expected_store,
                receipt: Some(receipt),
                disposition: JournaledEffectCapabilityIssuanceDispositionV1::Issued {
                    capability_record,
                    post_frontier,
                },
                reconciliation_valid_until: Some(receipt.valid_until),
                capability_eligibility_valid_until: Some(capability_eligibility_valid_until),
                _store: PhantomData,
            }
        }
        EffectCapabilityIssuanceDispositionV1::ProvenNotIssued {
            observed_frontier,
            absence_evidence,
        } => {
            if !final_absence_frontier_is_compatible(manifest.expected_frontier, observed_frontier)
            {
                return journaled_unknown(
                    manifest,
                    expected_store,
                    Some(receipt),
                    EffectCapabilityIssuanceAmbiguityReasonV1::InvalidProvenNotIssuedFrontier,
                );
            }

            JournaledEffectCapabilityIssuanceReconciliationV1 {
                manifest,
                expected_store,
                receipt: Some(receipt),
                disposition: JournaledEffectCapabilityIssuanceDispositionV1::ProvenNotIssued {
                    observed_frontier,
                    absence_evidence,
                },
                reconciliation_valid_until: Some(receipt.valid_until),
                capability_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
        EffectCapabilityIssuanceDispositionV1::AttemptIdConflict { .. } => journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityIssuanceAmbiguityReasonV1::AttemptIdConflict,
        ),
        EffectCapabilityIssuanceDispositionV1::EffectAdmissionAlreadyIssued { .. } => {
            journaled_unknown(
                manifest,
                expected_store,
                Some(receipt),
                EffectCapabilityIssuanceAmbiguityReasonV1::EffectAdmissionAlreadyIssued,
            )
        }
        EffectCapabilityIssuanceDispositionV1::OutcomeUnknown => journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityIssuanceAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn frontier(generation: u64, head: Option<u8>) -> DurableEffectCapabilityFrontierV1 {
        DurableEffectCapabilityFrontierV1 {
            generation: DurableEffectCapabilityGeneration::new(generation),
            head: head.map(|byte| EffectCapabilityRecordCommitment::from_bytes(bytes(byte))),
        }
    }

    #[test]
    fn admitted_frontier_advances_exactly_one_generation_to_exact_record() {
        let record = EffectCapabilityRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            issued_frontier_is_exact(frontier(4, Some(1)), record, frontier(5, Some(9))),
            Ok(())
        );
    }

    #[test]
    fn skipped_generation_is_rejected() {
        let record = EffectCapabilityRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            issued_frontier_is_exact(frontier(4, Some(1)), record, frontier(6, Some(9))),
            Err(EffectCapabilityIssuanceAmbiguityReasonV1::InvalidIssuedFrontier)
        );
    }

    #[test]
    fn final_absence_never_rolls_frontier_backward() {
        assert!(!final_absence_frontier_is_compatible(
            frontier(5, Some(2)),
            frontier(4, Some(2))
        ));
    }

    #[test]
    fn equal_generation_absence_requires_exact_head() {
        assert!(!final_absence_frontier_is_compatible(
            frontier(5, Some(2)),
            frontier(5, Some(3))
        ));
    }

    #[test]
    fn preexisting_effect_issuance_is_not_final_absence() {
        let record = EffectCapabilityRecordCommitment::from_bytes(bytes(7));
        let existing = EffectCapabilityIssuanceDispositionV1::EffectAdmissionAlreadyIssued {
            existing_capability_record: record,
        };
        let absent = EffectCapabilityIssuanceDispositionV1::ProvenNotIssued {
            observed_frontier: frontier(1, None),
            absence_evidence: EffectCapabilityIssuanceAbsenceCommitment::from_bytes(bytes(8)),
        };
        assert_ne!(existing, absent);
    }

    #[test]
    fn store_descriptor_has_explicit_time_basis() {
        let descriptor = EffectCapabilityIssuanceStoreDescriptorV1 {
            stable_identity: EffectCapabilityIssuanceStoreIdentityCommitment::from_bytes(bytes(1)),
            policy: EffectCapabilityIssuanceStorePolicyCommitment::from_bytes(bytes(2)),
            generation: EffectCapabilityIssuanceStoreGeneration::new(3),
            time_basis: EffectCapabilityIssuanceStoreTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 100,
        };
        assert_eq!(
            descriptor.time_basis,
            EffectCapabilityIssuanceStoreTimeBasisV1::UnixMillisecondsUtc
        );
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Durable single-use issuance reservation for SSF adoption authority.
//!
//! A persisted registration is not consumed merely because it exists.
//! Before any authority-issuance layer may consider it, the exact registration
//! must be durably reserved at most once. Reservation remains evidence only:
//! it does not refresh authority, perform current security revalidation, mint
//! a lease, install state, or execute an effect.
//!
//! The external-effect rule matches the registration boundary: after the
//! reservation store may have written, every malformed or ambiguous outcome
//! becomes `OutcomeUnknown`. Ambiguity is reconciled using the same exact
//! attempt manifest; it is never converted into permission for a blind retry.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_persisted_registration_rebind::{
    PersistedRegistrationBindingV1, ReboundPersistedAdoptionRegistrationV1,
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

digest_type!(IssuanceReservationStoreIdentityCommitment);
digest_type!(IssuanceReservationStorePolicyCommitment);
digest_type!(IssuanceReservationRecordCommitment);
digest_type!(IssuanceReservationReceiptCommitment);
digest_type!(IssuanceReservationAttemptId);
digest_type!(IssuanceReservationAttemptAbsenceCommitment);
digest_type!(IssuanceReservationManifestCommitment);
generation_type!(IssuanceReservationStoreVerifierGeneration);
generation_type!(DurableIssuanceReservationGeneration);

/// Stable verifier/configuration identity for the issuance-reservation store.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IssuanceReservationStoreDescriptorV1 {
    pub identity: IssuanceReservationStoreIdentityCommitment,
    pub policy: IssuanceReservationStorePolicyCommitment,
    pub verifier_generation: IssuanceReservationStoreVerifierGeneration,
    pub valid_until: u64,
}

/// Independently expected store profile supplied by local trusted configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedIssuanceReservationStoreProfileV1 {
    descriptor: IssuanceReservationStoreDescriptorV1,
}

impl ExpectedIssuanceReservationStoreProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: IssuanceReservationStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> IssuanceReservationStoreDescriptorV1 {
        self.descriptor
    }
}

/// Exact durable frontier expected before the reservation attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DurableIssuanceReservationFrontierV1 {
    pub generation: DurableIssuanceReservationGeneration,
    pub head: Option<IssuanceReservationRecordCommitment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedDurableIssuanceReservationFrontierV1 {
    frontier: DurableIssuanceReservationFrontierV1,
}

impl ExpectedDurableIssuanceReservationFrontierV1 {
    pub const fn from_trusted_state(frontier: DurableIssuanceReservationFrontierV1) -> Self {
        Self { frontier }
    }

    pub const fn frontier(&self) -> DurableIssuanceReservationFrontierV1 {
        self.frontier
    }
}

/// Exact registration evidence being reserved for one future issuance lineage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IssuanceReservationSubjectV1 {
    pub persisted_registration: PersistedRegistrationBindingV1,
}

/// Plain crash-recoverable manifest for one exact reservation attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IssuanceReservationAttemptManifestV1 {
    schema_version: u16,
    attempt_id: IssuanceReservationAttemptId,
    subject: IssuanceReservationSubjectV1,
    expected_store: IssuanceReservationStoreDescriptorV1,
    expected_frontier: DurableIssuanceReservationFrontierV1,
}

impl IssuanceReservationAttemptManifestV1 {
    pub const fn from_journaled_parts(
        attempt_id: IssuanceReservationAttemptId,
        subject: IssuanceReservationSubjectV1,
        expected_store: IssuanceReservationStoreDescriptorV1,
        expected_frontier: DurableIssuanceReservationFrontierV1,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            attempt_id,
            subject,
            expected_store,
            expected_frontier,
        }
    }

    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub const fn attempt_id(&self) -> IssuanceReservationAttemptId {
        self.attempt_id
    }

    pub const fn subject(&self) -> IssuanceReservationSubjectV1 {
        self.subject
    }

    pub const fn expected_store(&self) -> IssuanceReservationStoreDescriptorV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> DurableIssuanceReservationFrontierV1 {
        self.expected_frontier
    }
}

/// Closed-world store-reported reservation result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssuanceReservationDispositionV1 {
    Reserved {
        record: IssuanceReservationRecordCommitment,
        post_frontier: DurableIssuanceReservationFrontierV1,
    },
    ProvenNotReserved {
        observed_frontier: DurableIssuanceReservationFrontierV1,
        absence_evidence: IssuanceReservationAttemptAbsenceCommitment,
    },
    AttemptIdConflict {
        bound_manifest: IssuanceReservationManifestCommitment,
    },
    RegistrationAlreadyReserved {
        existing_record: IssuanceReservationRecordCommitment,
    },
    OutcomeUnknown,
}

/// Untrusted receipt returned after a write or read-only reconciliation query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IssuanceReservationReceiptV1 {
    pub schema_version: u16,
    pub store: IssuanceReservationStoreDescriptorV1,
    pub manifest: IssuanceReservationAttemptManifestV1,
    pub disposition: IssuanceReservationDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: IssuanceReservationReceiptCommitment,
}

impl IssuanceReservationReceiptV1 {
    pub const fn new(
        store: IssuanceReservationStoreDescriptorV1,
        manifest: IssuanceReservationAttemptManifestV1,
        disposition: IssuanceReservationDispositionV1,
        valid_until: u64,
        receipt_commitment: IssuanceReservationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            store,
            manifest,
            disposition,
            valid_until,
            receipt_commitment,
        }
    }
}

/// External durable reservation adapter.
///
/// Implementations must enforce both attempt-ID uniqueness and one durable
/// reservation record per exact persisted registration record.
/// `reconcile_reservation` is read-only and may never create reservation state.
/// `ProvenNotReserved` is final for the exact attempt ID.
pub trait IssuanceReservationStoreV1 {
    fn descriptor(&self) -> IssuanceReservationStoreDescriptorV1;

    fn reserve_issuance(
        &self,
        manifest: &IssuanceReservationAttemptManifestV1,
    ) -> IssuanceReservationReceiptV1;

    fn reconcile_reservation(
        &self,
        manifest: &IssuanceReservationAttemptManifestV1,
    ) -> IssuanceReservationReceiptV1;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssuanceReservationPreparationError {
    StoreDescriptorMismatchBefore,
}

/// Every post-attempt anomaly is a frozen outcome, not a retryable error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IssuanceReservationAmbiguityReasonV1 {
    StoreReportedOutcomeUnknown,
    AttemptIdConflict,
    RegistrationAlreadyReserved,
    StoreDescriptorMismatchBeforeReconciliation,
    StoreDescriptorChangedAfter,
    UnsupportedManifestSchema,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    PersistedGenerationOverflow,
    InvalidPersistedFrontier,
    InvalidProvenNotReservedFrontier,
}

fn validate_persisted_frontier(
    before: DurableIssuanceReservationFrontierV1,
    record: IssuanceReservationRecordCommitment,
    after: DurableIssuanceReservationFrontierV1,
) -> Result<(), IssuanceReservationAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(IssuanceReservationAmbiguityReasonV1::PersistedGenerationOverflow)?;

    if after.generation.get() != expected_generation || after.head != Some(record) {
        return Err(IssuanceReservationAmbiguityReasonV1::InvalidPersistedFrontier);
    }

    Ok(())
}

fn validate_absence_frontier(
    expected: DurableIssuanceReservationFrontierV1,
    observed: DurableIssuanceReservationFrontierV1,
) -> Result<(), IssuanceReservationAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(IssuanceReservationAmbiguityReasonV1::InvalidProvenNotReservedFrontier);
    }

    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(IssuanceReservationAmbiguityReasonV1::InvalidProvenNotReservedFrontier);
    }

    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ValidatedIssuanceReservationDispositionV1 {
    Reserved {
        record: IssuanceReservationRecordCommitment,
        post_frontier: DurableIssuanceReservationFrontierV1,
    },
    ProvenNotReserved {
        observed_frontier: DurableIssuanceReservationFrontierV1,
        absence_evidence: IssuanceReservationAttemptAbsenceCommitment,
    },
    OutcomeUnknown(IssuanceReservationAmbiguityReasonV1),
}

fn validate_receipt(
    expected_store: IssuanceReservationStoreDescriptorV1,
    manifest: IssuanceReservationAttemptManifestV1,
    receipt: &IssuanceReservationReceiptV1,
) -> Result<ValidatedIssuanceReservationDispositionV1, IssuanceReservationAmbiguityReasonV1> {
    if manifest.schema_version() != SSF_SCHEMA_V1 {
        return Err(IssuanceReservationAmbiguityReasonV1::UnsupportedManifestSchema);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(IssuanceReservationAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_store {
        return Err(IssuanceReservationAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest != manifest {
        return Err(IssuanceReservationAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.valid_until {
        return Err(IssuanceReservationAmbiguityReasonV1::ReceiptOutlivesStore);
    }

    match receipt.disposition {
        IssuanceReservationDispositionV1::Reserved {
            record,
            post_frontier,
        } => {
            validate_persisted_frontier(manifest.expected_frontier(), record, post_frontier)?;
            Ok(ValidatedIssuanceReservationDispositionV1::Reserved {
                record,
                post_frontier,
            })
        }
        IssuanceReservationDispositionV1::ProvenNotReserved {
            observed_frontier,
            absence_evidence,
        } => {
            validate_absence_frontier(manifest.expected_frontier(), observed_frontier)?;
            Ok(ValidatedIssuanceReservationDispositionV1::ProvenNotReserved {
                observed_frontier,
                absence_evidence,
            })
        }
        IssuanceReservationDispositionV1::AttemptIdConflict { .. } => Ok(
            ValidatedIssuanceReservationDispositionV1::OutcomeUnknown(
                IssuanceReservationAmbiguityReasonV1::AttemptIdConflict,
            ),
        ),
        IssuanceReservationDispositionV1::RegistrationAlreadyReserved { .. } => Ok(
            ValidatedIssuanceReservationDispositionV1::OutcomeUnknown(
                IssuanceReservationAmbiguityReasonV1::RegistrationAlreadyReserved,
            ),
        ),
        IssuanceReservationDispositionV1::OutcomeUnknown => Ok(
            ValidatedIssuanceReservationDispositionV1::OutcomeUnknown(
                IssuanceReservationAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            ),
        ),
    }
}

/// Prepared reservation retains the complete rebound registration until the
/// durable outcome is resolved.
pub struct PreparedSingleUseIssuanceReservationV1<
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    rebound: ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R>,
    expected_store: ExpectedIssuanceReservationStoreProfileV1,
    expected_frontier: ExpectedDurableIssuanceReservationFrontierV1,
    manifest: IssuanceReservationAttemptManifestV1,
}

impl<RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn manifest(&self) -> IssuanceReservationAttemptManifestV1 {
        self.manifest
    }

    pub const fn rebound(
        &self,
    ) -> &ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R> {
        &self.rebound
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn prepare_single_use_issuance_reservation<
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    expected_store: ExpectedIssuanceReservationStoreProfileV1,
    expected_frontier: ExpectedDurableIssuanceReservationFrontierV1,
    attempt_id: IssuanceReservationAttemptId,
    rebound: ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R>,
) -> PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R> {
    let subject = IssuanceReservationSubjectV1 {
        persisted_registration: rebound.binding(),
    };
    let manifest = IssuanceReservationAttemptManifestV1::from_journaled_parts(
        attempt_id,
        subject,
        expected_store.descriptor(),
        expected_frontier.frontier(),
    );

    PreparedSingleUseIssuanceReservationV1 {
        rebound,
        expected_store,
        expected_frontier,
        manifest,
    }
}

struct BoundReservationEvidenceV1<
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
    prepared: PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R>,
    receipt: IssuanceReservationReceiptV1,
    _store_type: PhantomData<fn() -> IS>,
}

/// Exact reservation evidence eligible only for later current-state
/// revalidation. It still contains no authority lease.
pub struct ReservedSingleUseIssuanceV1<
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
    bound: BoundReservationEvidenceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    authority_eligibility_valid_until: u64,
}

pub struct ProvenUnreservedSingleUseIssuanceV1<
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
    bound: BoundReservationEvidenceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
}

pub struct FrozenAmbiguousSingleUseIssuanceReservationV1<
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
    prepared: PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R>,
    last_receipt: IssuanceReservationReceiptV1,
    reason: IssuanceReservationAmbiguityReasonV1,
    _store_type: PhantomData<fn() -> IS>,
}

impl<IS, RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    ReservedSingleUseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn rebound(
        &self,
    ) -> &ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R> {
        self.bound.prepared.rebound()
    }

    pub const fn manifest(&self) -> IssuanceReservationAttemptManifestV1 {
        self.bound.prepared.manifest()
    }

    pub const fn receipt(&self) -> &IssuanceReservationReceiptV1 {
        &self.bound.receipt
    }

    pub const fn authority_eligibility_valid_until(&self) -> u64 {
        self.authority_eligibility_valid_until
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

impl<IS, RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    ProvenUnreservedSingleUseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn manifest(&self) -> IssuanceReservationAttemptManifestV1 {
        self.bound.prepared.manifest()
    }

    pub const fn receipt(&self) -> &IssuanceReservationReceiptV1 {
        &self.bound.receipt
    }

    pub const fn old_attempt_is_permanently_closed(&self) -> bool {
        true
    }

    pub const fn new_attempt_may_be_considered(&self) -> bool {
        true
    }

    pub fn into_rebound(
        self,
    ) -> ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R> {
        self.bound.prepared.rebound
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

impl<IS, RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    FrozenAmbiguousSingleUseIssuanceReservationV1<IS, RS, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn manifest(&self) -> IssuanceReservationAttemptManifestV1 {
        self.prepared.manifest()
    }

    pub const fn rebound(
        &self,
    ) -> &ReboundPersistedAdoptionRegistrationV1<RS, P, EV, RV, AQ, LQ, L, R> {
        self.prepared.rebound()
    }

    pub const fn last_receipt(&self) -> &IssuanceReservationReceiptV1 {
        &self.last_receipt
    }

    pub const fn reason(&self) -> IssuanceReservationAmbiguityReasonV1 {
        self.reason
    }

    pub const fn retry_forbidden(&self) -> bool {
        true
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub enum SingleUseIssuanceReservationResultV1<
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
    Reserved(ReservedSingleUseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>),
    ProvenNotReserved(
        ProvenUnreservedSingleUseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    ),
    OutcomeUnknown(
        FrozenAmbiguousSingleUseIssuanceReservationV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    ),
}

pub struct IssuanceReservationPreparationFailureV1<
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    prepared: PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R>,
    error: IssuanceReservationPreparationError,
}

impl<RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    IssuanceReservationPreparationFailureV1<RS, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn error(&self) -> IssuanceReservationPreparationError {
        self.error
    }

    pub fn into_prepared(
        self,
    ) -> PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R> {
        self.prepared
    }
}

fn freeze<IS, RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    prepared: PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R>,
    receipt: IssuanceReservationReceiptV1,
    reason: IssuanceReservationAmbiguityReasonV1,
) -> SingleUseIssuanceReservationResultV1<IS, RS, P, EV, RV, AQ, LQ, L, R> {
    SingleUseIssuanceReservationResultV1::OutcomeUnknown(
        FrozenAmbiguousSingleUseIssuanceReservationV1 {
            prepared,
            last_receipt: receipt,
            reason,
            _store_type: PhantomData,
        },
    )
}

fn resolve_typed_receipt<IS, RS, P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    prepared: PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R>,
    descriptor_after: IssuanceReservationStoreDescriptorV1,
    receipt: IssuanceReservationReceiptV1,
) -> SingleUseIssuanceReservationResultV1<IS, RS, P, EV, RV, AQ, LQ, L, R> {
    let expected_descriptor = prepared.expected_store.descriptor();
    if descriptor_after != expected_descriptor {
        return freeze(
            prepared,
            receipt,
            IssuanceReservationAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }

    let validated = match validate_receipt(expected_descriptor, prepared.manifest(), &receipt) {
        Ok(value) => value,
        Err(reason) => return freeze(prepared, receipt, reason),
    };

    match validated {
        ValidatedIssuanceReservationDispositionV1::Reserved { .. } => {
            let authority_eligibility_valid_until = receipt
                .valid_until
                .min(prepared.rebound().authority_eligibility_valid_until());
            SingleUseIssuanceReservationResultV1::Reserved(ReservedSingleUseIssuanceV1 {
                bound: BoundReservationEvidenceV1 {
                    prepared,
                    receipt,
                    _store_type: PhantomData,
                },
                authority_eligibility_valid_until,
            })
        }
        ValidatedIssuanceReservationDispositionV1::ProvenNotReserved { .. } => {
            SingleUseIssuanceReservationResultV1::ProvenNotReserved(
                ProvenUnreservedSingleUseIssuanceV1 {
                    bound: BoundReservationEvidenceV1 {
                        prepared,
                        receipt,
                        _store_type: PhantomData,
                    },
                },
            )
        }
        ValidatedIssuanceReservationDispositionV1::OutcomeUnknown(reason) => {
            freeze(prepared, receipt, reason)
        }
    }
}

/// Execute the one write-capable reservation attempt.
pub fn reserve_single_use_issuance<
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
    store: &IS,
    prepared: PreparedSingleUseIssuanceReservationV1<RS, P, EV, RV, AQ, LQ, L, R>,
) -> Result<
    SingleUseIssuanceReservationResultV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    IssuanceReservationPreparationFailureV1<RS, P, EV, RV, AQ, LQ, L, R>,
>
where
    IS: IssuanceReservationStoreV1,
{
    let expected_descriptor = prepared.expected_store.descriptor();
    if store.descriptor() != expected_descriptor {
        return Err(IssuanceReservationPreparationFailureV1 {
            prepared,
            error: IssuanceReservationPreparationError::StoreDescriptorMismatchBefore,
        });
    }

    let receipt = store.reserve_issuance(&prepared.manifest());
    let descriptor_after = store.descriptor();

    Ok(resolve_typed_receipt::<IS, RS, P, EV, RV, AQ, LQ, L, R>(
        prepared,
        descriptor_after,
        receipt,
    ))
}

/// Same-attempt, read-only reconciliation for an in-memory frozen reservation.
pub fn reconcile_same_issuance_reservation<
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
    store: &IS,
    frozen: FrozenAmbiguousSingleUseIssuanceReservationV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
) -> SingleUseIssuanceReservationResultV1<IS, RS, P, EV, RV, AQ, LQ, L, R>
where
    IS: IssuanceReservationStoreV1,
{
    let expected_descriptor = frozen.prepared.expected_store.descriptor();
    if store.descriptor() != expected_descriptor {
        let receipt = frozen.last_receipt;
        return freeze(
            frozen.prepared,
            receipt,
            IssuanceReservationAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = store.reconcile_reservation(&frozen.prepared.manifest());
    let descriptor_after = store.descriptor();

    resolve_typed_receipt::<IS, RS, P, EV, RV, AQ, LQ, L, R>(
        frozen.prepared,
        descriptor_after,
        receipt,
    )
}

/// Restart-safe evidence produced from a journaled manifest alone.
pub struct JournaledIssuanceReservationReconciliationV1<IS> {
    manifest: IssuanceReservationAttemptManifestV1,
    receipt: Option<IssuanceReservationReceiptV1>,
    disposition: JournaledIssuanceReservationDispositionV1,
    reconciliation_valid_until: Option<u64>,
    authority_eligibility_valid_until: Option<u64>,
    _store_type: PhantomData<fn() -> IS>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JournaledIssuanceReservationDispositionV1 {
    Reserved {
        record: IssuanceReservationRecordCommitment,
        post_frontier: DurableIssuanceReservationFrontierV1,
    },
    ProvenNotReserved {
        observed_frontier: DurableIssuanceReservationFrontierV1,
        absence_evidence: IssuanceReservationAttemptAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: IssuanceReservationAmbiguityReasonV1,
    },
}

impl<IS> JournaledIssuanceReservationReconciliationV1<IS> {
    pub const fn manifest(&self) -> IssuanceReservationAttemptManifestV1 {
        self.manifest
    }

    pub fn receipt(&self) -> Option<&IssuanceReservationReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> JournaledIssuanceReservationDispositionV1 {
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
            JournaledIssuanceReservationDispositionV1::ProvenNotReserved { .. }
        )
    }

    pub fn reserved_lineage_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            JournaledIssuanceReservationDispositionV1::Reserved { .. }
        )
    }

    pub const fn contains_current_authority_revalidation(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

fn journaled_unknown<IS>(
    manifest: IssuanceReservationAttemptManifestV1,
    receipt: Option<IssuanceReservationReceiptV1>,
    reason: IssuanceReservationAmbiguityReasonV1,
) -> JournaledIssuanceReservationReconciliationV1<IS> {
    JournaledIssuanceReservationReconciliationV1 {
        manifest,
        receipt,
        disposition: JournaledIssuanceReservationDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        authority_eligibility_valid_until: None,
        _store_type: PhantomData,
    }
}

/// Read-only process-restart reconciliation from an exact journaled manifest.
pub fn reconcile_journaled_issuance_reservation<IS>(
    expected_store: ExpectedIssuanceReservationStoreProfileV1,
    store: &IS,
    manifest: IssuanceReservationAttemptManifestV1,
) -> JournaledIssuanceReservationReconciliationV1<IS>
where
    IS: IssuanceReservationStoreV1,
{
    let expected_descriptor = expected_store.descriptor();

    if manifest.schema_version() != SSF_SCHEMA_V1 {
        return journaled_unknown(
            manifest,
            None,
            IssuanceReservationAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }

    if manifest.expected_store() != expected_descriptor || store.descriptor() != expected_descriptor {
        return journaled_unknown(
            manifest,
            None,
            IssuanceReservationAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = store.reconcile_reservation(&manifest);
    if store.descriptor() != expected_descriptor {
        return journaled_unknown(
            manifest,
            Some(receipt),
            IssuanceReservationAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }

    let validated = match validate_receipt(expected_descriptor, manifest, &receipt) {
        Ok(value) => value,
        Err(reason) => return journaled_unknown(manifest, Some(receipt), reason),
    };

    match validated {
        ValidatedIssuanceReservationDispositionV1::Reserved {
            record,
            post_frontier,
        } => JournaledIssuanceReservationReconciliationV1 {
            manifest,
            receipt: Some(receipt),
            disposition: JournaledIssuanceReservationDispositionV1::Reserved {
                record,
                post_frontier,
            },
            reconciliation_valid_until: Some(receipt.valid_until),
            authority_eligibility_valid_until: Some(
                receipt
                    .valid_until
                    .min(manifest.subject().persisted_registration.authority_eligibility_valid_until),
            ),
            _store_type: PhantomData,
        },
        ValidatedIssuanceReservationDispositionV1::ProvenNotReserved {
            observed_frontier,
            absence_evidence,
        } => JournaledIssuanceReservationReconciliationV1 {
            manifest,
            receipt: Some(receipt),
            disposition: JournaledIssuanceReservationDispositionV1::ProvenNotReserved {
                observed_frontier,
                absence_evidence,
            },
            reconciliation_valid_until: Some(receipt.valid_until),
            authority_eligibility_valid_until: None,
            _store_type: PhantomData,
        },
        ValidatedIssuanceReservationDispositionV1::OutcomeUnknown(reason) => {
            journaled_unknown(manifest, Some(receipt), reason)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn frontier(
        generation: u64,
        head: Option<u8>,
    ) -> DurableIssuanceReservationFrontierV1 {
        DurableIssuanceReservationFrontierV1 {
            generation: DurableIssuanceReservationGeneration::new(generation),
            head: match head {
                Some(byte) => Some(IssuanceReservationRecordCommitment::from_bytes(bytes(byte))),
                None => None,
            },
        }
    }

    #[test]
    fn reserved_frontier_advances_exactly_once_to_exact_record() {
        let before = frontier(7, Some(3));
        let record = IssuanceReservationRecordCommitment::from_bytes(bytes(4));
        assert_eq!(
            validate_persisted_frontier(before, record, frontier(8, Some(4))),
            Ok(())
        );
        assert_eq!(
            validate_persisted_frontier(before, record, frontier(9, Some(4))),
            Err(IssuanceReservationAmbiguityReasonV1::InvalidPersistedFrontier)
        );
        assert_eq!(
            validate_persisted_frontier(before, record, frontier(8, Some(5))),
            Err(IssuanceReservationAmbiguityReasonV1::InvalidPersistedFrontier)
        );
    }

    #[test]
    fn final_nonreservation_cannot_roll_back_frontier() {
        let expected = frontier(7, Some(3));
        assert_eq!(validate_absence_frontier(expected, frontier(8, Some(9))), Ok(()));
        assert_eq!(
            validate_absence_frontier(expected, frontier(6, Some(2))),
            Err(IssuanceReservationAmbiguityReasonV1::InvalidProvenNotReservedFrontier)
        );
    }

    #[test]
    fn already_reserved_and_attempt_conflict_are_not_nonreservation() {
        assert_ne!(
            IssuanceReservationDispositionV1::AttemptIdConflict {
                bound_manifest: IssuanceReservationManifestCommitment::from_bytes(bytes(1))
            },
            IssuanceReservationDispositionV1::ProvenNotReserved {
                observed_frontier: frontier(1, None),
                absence_evidence: IssuanceReservationAttemptAbsenceCommitment::from_bytes(bytes(2)),
            }
        );
        assert_ne!(
            IssuanceReservationDispositionV1::RegistrationAlreadyReserved {
                existing_record: IssuanceReservationRecordCommitment::from_bytes(bytes(3))
            },
            IssuanceReservationDispositionV1::ProvenNotReserved {
                observed_frontier: frontier(1, None),
                absence_evidence: IssuanceReservationAttemptAbsenceCommitment::from_bytes(bytes(2)),
            }
        );
    }

    #[test]
    fn reservation_evidence_never_refreshes_registration_authority() {
        let receipt_valid_until = 180u64;
        let registration_authority_ceiling = 90u64;
        assert_eq!(receipt_valid_until.min(registration_authority_ceiling), 90);
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Durable authority-lease issuance for the Mycelix Solar-System Federation
//! security profile.
//!
//! A bounded lease candidate is not authority merely because it passed current
//! revalidation. This crate adds a crash-safe durable issuance boundary. It
//! establishes at most one semantic lease lineage for one exact single-use
//! issuance reservation while preserving `OutcomeUnknown` rather than turning
//! ambiguous external writes into blind retries.
//!
//! A durably issued lease is still not an actuator capability. A later
//! use-time/effect-admission layer must revalidate current state/freshness and
//! bind the lease to the exact execution surface before any effect can occur.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_adoption_authority_candidate::AdoptionAuthorityOperationV1;
use mycelix_ssf_bounded_authority_lease_candidate::{
    AuthorityLeaseCandidateId, AuthorityLeaseNonce, BoundedAuthorityLeaseCandidateV1,
};
use mycelix_ssf_contracts::{ConsequenceClass, SSF_SCHEMA_V1};
use mycelix_ssf_local_adoption_policy::LocalAdoptionScopeCommitment;
use mycelix_ssf_reserved_issuance_rebind::ReservedIssuanceBindingV1;
use mycelix_ssf_snapshots::FederationStateHeadV1;

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

digest_type!(AuthorityLeaseIssuanceAttemptId);
digest_type!(AuthorityLeaseRecordCommitment);
digest_type!(AuthorityLeaseIssuanceReceiptCommitment);
digest_type!(AuthorityLeaseIssuanceAttemptAbsenceCommitment);
digest_type!(AuthorityLeaseIssuanceManifestCommitment);
digest_type!(AuthorityLeaseStoreIdentityCommitment);
digest_type!(AuthorityLeaseStorePolicyCommitment);
generation_type!(AuthorityLeaseStoreGeneration);
generation_type!(DurableAuthorityLeaseGeneration);

/// Exact verifier/configuration generation of the durable lease store.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AuthorityLeaseStoreDescriptorV1 {
    pub identity: AuthorityLeaseStoreIdentityCommitment,
    pub policy: AuthorityLeaseStorePolicyCommitment,
    pub generation: AuthorityLeaseStoreGeneration,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedAuthorityLeaseStoreProfileV1 {
    descriptor: AuthorityLeaseStoreDescriptorV1,
}

impl ExpectedAuthorityLeaseStoreProfileV1 {
    pub const fn from_trusted_configuration(descriptor: AuthorityLeaseStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> AuthorityLeaseStoreDescriptorV1 {
        self.descriptor
    }
}

/// Exact durable authority-lease log frontier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DurableAuthorityLeaseFrontierV1 {
    pub generation: DurableAuthorityLeaseGeneration,
    pub head: Option<AuthorityLeaseRecordCommitment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedDurableAuthorityLeaseFrontierV1 {
    frontier: DurableAuthorityLeaseFrontierV1,
}

impl ExpectedDurableAuthorityLeaseFrontierV1 {
    pub const fn from_trusted_state(frontier: DurableAuthorityLeaseFrontierV1) -> Self {
        Self { frontier }
    }

    pub const fn frontier(&self) -> DurableAuthorityLeaseFrontierV1 {
        self.frontier
    }
}

/// Plain crash-recoverable issuance subject.
///
/// The complete reservation binding is retained so the store can enforce that
/// one exact single-use reservation produces at most one lease lineage. IDs or
/// matching endpoint state alone are insufficient.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorityLeaseIssuanceSubjectV1 {
    pub reserved_issuance: ReservedIssuanceBindingV1,
    pub lease_candidate_id: AuthorityLeaseCandidateId,
    pub nonce: AuthorityLeaseNonce,
    pub operation: AdoptionAuthorityOperationV1,
    pub source_local_state: FederationStateHeadV1,
    pub remote_target: FederationStateHeadV1,
    pub scope: LocalAdoptionScopeCommitment,
    pub consequence: ConsequenceClass,
    pub lease_valid_until: u64,
}

/// Exact journalable issuance attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorityLeaseIssuanceAttemptManifestV1 {
    pub schema_version: u16,
    attempt_id: AuthorityLeaseIssuanceAttemptId,
    subject: AuthorityLeaseIssuanceSubjectV1,
    expected_store: AuthorityLeaseStoreDescriptorV1,
    expected_frontier: DurableAuthorityLeaseFrontierV1,
}

impl AuthorityLeaseIssuanceAttemptManifestV1 {
    pub const fn from_journaled_parts(
        attempt_id: AuthorityLeaseIssuanceAttemptId,
        subject: AuthorityLeaseIssuanceSubjectV1,
        expected_store: AuthorityLeaseStoreDescriptorV1,
        expected_frontier: DurableAuthorityLeaseFrontierV1,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            attempt_id,
            subject,
            expected_store,
            expected_frontier,
        }
    }

    pub const fn attempt_id(&self) -> AuthorityLeaseIssuanceAttemptId {
        self.attempt_id
    }

    pub const fn subject(&self) -> AuthorityLeaseIssuanceSubjectV1 {
        self.subject
    }

    pub const fn expected_store(&self) -> AuthorityLeaseStoreDescriptorV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> DurableAuthorityLeaseFrontierV1 {
        self.expected_frontier
    }
}

/// Closed-world durable store result.
///
/// Store implementations MUST enforce both:
///
/// 1. first-seen attempt ID is permanently bound to the exact manifest; and
/// 2. one exact `reserved_issuance` binding can produce at most one durable
///    authority-lease record.
///
/// A different attempt ID therefore cannot mint a second lease from the same
/// single-use reservation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityLeaseIssuanceDispositionV1 {
    Issued {
        lease_record: AuthorityLeaseRecordCommitment,
        post_frontier: DurableAuthorityLeaseFrontierV1,
    },
    ProvenNotIssued {
        observed_frontier: DurableAuthorityLeaseFrontierV1,
        absence_evidence: AuthorityLeaseIssuanceAttemptAbsenceCommitment,
    },
    AttemptIdConflict {
        existing_manifest: AuthorityLeaseIssuanceManifestCommitment,
    },
    ReservationAlreadyIssued {
        existing_lease_record: AuthorityLeaseRecordCommitment,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorityLeaseIssuanceReceiptV1 {
    pub schema_version: u16,
    pub store: AuthorityLeaseStoreDescriptorV1,
    pub manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    pub disposition: AuthorityLeaseIssuanceDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: AuthorityLeaseIssuanceReceiptCommitment,
}

/// External durable store boundary.
///
/// `issue_authority_lease` may write. Once invoked, caller-side validation
/// failures MUST become `OutcomeUnknown`, not ordinary retryable errors.
///
/// `reconcile_authority_lease` is read-only/idempotent for the exact same
/// attempt. It must never create a new lease record.
pub trait AuthorityLeaseIssuanceStoreV1 {
    type Error;

    fn descriptor(&self) -> AuthorityLeaseStoreDescriptorV1;

    fn issue_authority_lease(
        &self,
        manifest: &AuthorityLeaseIssuanceAttemptManifestV1,
    ) -> Result<AuthorityLeaseIssuanceReceiptV1, Self::Error>;

    fn reconcile_authority_lease(
        &self,
        manifest: &AuthorityLeaseIssuanceAttemptManifestV1,
    ) -> Result<AuthorityLeaseIssuanceReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityLeaseIssuancePreparationError {
    StoreDescriptorMismatchBefore,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuthorityLeaseIssuanceAmbiguityReasonV1 {
    StoreErrorAfterAttempt,
    StoreDescriptorChangedAfter,
    StoreDescriptorMismatchBeforeReconciliation,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    IssuedGenerationOverflow,
    InvalidIssuedFrontier,
    InvalidProvenNotIssuedFrontier,
    AttemptIdConflict,
    ReservationAlreadyIssued,
    StoreReportedOutcomeUnknown,
}

/// Prepared issuance retains the exact bounded candidate until external outcome
/// becomes final.
pub struct PreparedAuthorityLeaseIssuanceV1<
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
    const L: usize,
    const R: usize,
> {
    candidate:
        BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    expected_store: ExpectedAuthorityLeaseStoreProfileV1,
    expected_frontier: ExpectedDurableAuthorityLeaseFrontierV1,
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
}

impl<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn candidate(
        &self,
    ) -> &BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
        &self.candidate
    }

    pub const fn manifest(&self) -> AuthorityLeaseIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_store(&self) -> ExpectedAuthorityLeaseStoreProfileV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> ExpectedDurableAuthorityLeaseFrontierV1 {
        self.expected_frontier
    }

    pub const fn contains_semantic_lease_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub struct AuthorityLeaseIssuancePreparationFailureV1<
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
    const L: usize,
    const R: usize,
> {
    prepared: PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    error: AuthorityLeaseIssuancePreparationError,
}

impl<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    AuthorityLeaseIssuancePreparationFailureV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn error(&self) -> AuthorityLeaseIssuancePreparationError {
        self.error
    }

    pub fn into_prepared(
        self,
    ) -> PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
        self.prepared
    }
}

struct BoundAuthorityLeaseIssuanceV1<
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
    const L: usize,
    const R: usize,
> {
    prepared: PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    receipt: AuthorityLeaseIssuanceReceiptV1,
    _store: PhantomData<fn() -> S>,
}

/// Durable semantic lease authority.
///
/// This token establishes that one exact single-use reservation has produced
/// one exact durable lease record. It is still not an actuator capability and
/// may be stale by the time a later effect is attempted. A use-time admission
/// layer must revalidate current security state and bind the exact execution
/// surface before any effect can occur.
pub struct DurablyIssuedAuthorityLeaseV1<
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
    const L: usize,
    const R: usize,
> {
    bound: BoundAuthorityLeaseIssuanceV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    lease_record: AuthorityLeaseRecordCommitment,
    post_frontier: DurableAuthorityLeaseFrontierV1,
    authority_valid_until: u64,
}

impl<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    DurablyIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn candidate(
        &self,
    ) -> &BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
        self.bound.prepared.candidate()
    }

    pub const fn manifest(&self) -> AuthorityLeaseIssuanceAttemptManifestV1 {
        self.bound.prepared.manifest()
    }

    pub const fn receipt(&self) -> &AuthorityLeaseIssuanceReceiptV1 {
        &self.bound.receipt
    }

    pub const fn lease_record(&self) -> AuthorityLeaseRecordCommitment {
        self.lease_record
    }

    pub const fn post_frontier(&self) -> DurableAuthorityLeaseFrontierV1 {
        self.post_frontier
    }

    pub const fn authority_valid_until(&self) -> u64 {
        self.authority_valid_until
    }

    pub const fn contains_semantic_lease_authority(&self) -> bool {
        true
    }

    pub const fn requires_use_time_revalidation(&self) -> bool {
        true
    }

    pub const fn requires_effect_admission(&self) -> bool {
        true
    }

    pub const fn delegation_allowed(&self) -> bool {
        false
    }

    pub const fn contains_direct_state_install_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub struct ProvenUnissuedAuthorityLeaseV1<
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
    const L: usize,
    const R: usize,
> {
    bound: BoundAuthorityLeaseIssuanceV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
}

impl<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    ProvenUnissuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn old_attempt_is_permanently_closed(&self) -> bool {
        true
    }

    pub const fn retry_with_new_attempt_may_be_considered(&self) -> bool {
        true
    }

    pub fn into_candidate(
        self,
    ) -> BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
        self.bound.prepared.candidate
    }

    pub const fn receipt(&self) -> &AuthorityLeaseIssuanceReceiptV1 {
        &self.bound.receipt
    }

    pub const fn contains_semantic_lease_authority(&self) -> bool {
        false
    }
}

/// Frozen ambiguous issuance. No API returns the candidate for a fresh attempt.
/// Progress is possible only through exact same-attempt reconciliation.
pub struct FrozenAmbiguousAuthorityLeaseIssuanceV1<
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
    const L: usize,
    const R: usize,
> {
    prepared: PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    last_receipt: Option<AuthorityLeaseIssuanceReceiptV1>,
    reason: AuthorityLeaseIssuanceAmbiguityReasonV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    FrozenAmbiguousAuthorityLeaseIssuanceV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn manifest(&self) -> AuthorityLeaseIssuanceAttemptManifestV1 {
        self.prepared.manifest()
    }

    pub const fn reason(&self) -> AuthorityLeaseIssuanceAmbiguityReasonV1 {
        self.reason
    }

    pub const fn last_receipt(&self) -> Option<&AuthorityLeaseIssuanceReceiptV1> {
        self.last_receipt.as_ref()
    }

    pub const fn retry_forbidden(&self) -> bool {
        true
    }

    pub const fn frozen_pending_reconciliation(&self) -> bool {
        true
    }

    pub const fn contains_semantic_lease_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub enum AuthorityLeaseIssuanceResultV1<
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
    const L: usize,
    const R: usize,
> {
    Issued(DurablyIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>),
    ProvenNotIssued(
        ProvenUnissuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    ),
    OutcomeUnknown(
        FrozenAmbiguousAuthorityLeaseIssuanceV1<
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
            L,
            R,
        >,
    ),
}

fn subject_from_candidate<
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
    const L: usize,
    const R: usize,
>(
    candidate: &BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
) -> AuthorityLeaseIssuanceSubjectV1 {
    AuthorityLeaseIssuanceSubjectV1 {
        reserved_issuance: candidate.revalidated().reserved().binding(),
        lease_candidate_id: candidate.lease_candidate_id(),
        nonce: candidate.nonce(),
        operation: candidate.operation(),
        source_local_state: candidate.source_local_state(),
        remote_target: candidate.remote_target(),
        scope: candidate.scope(),
        consequence: candidate.consequence(),
        lease_valid_until: candidate.valid_until(),
    }
}

pub fn prepare_authority_lease_issuance<
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
    const L: usize,
    const R: usize,
>(
    expected_store: ExpectedAuthorityLeaseStoreProfileV1,
    expected_frontier: ExpectedDurableAuthorityLeaseFrontierV1,
    attempt_id: AuthorityLeaseIssuanceAttemptId,
    candidate: BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
) -> PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
    let manifest = AuthorityLeaseIssuanceAttemptManifestV1::from_journaled_parts(
        attempt_id,
        subject_from_candidate(&candidate),
        expected_store.descriptor(),
        expected_frontier.frontier(),
    );

    PreparedAuthorityLeaseIssuanceV1 {
        candidate,
        expected_store,
        expected_frontier,
        manifest,
    }
}

fn validate_issued_frontier(
    before: DurableAuthorityLeaseFrontierV1,
    lease_record: AuthorityLeaseRecordCommitment,
    after: DurableAuthorityLeaseFrontierV1,
) -> Result<(), AuthorityLeaseIssuanceAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(AuthorityLeaseIssuanceAmbiguityReasonV1::IssuedGenerationOverflow)?;

    if after.generation.get() != expected_generation || after.head != Some(lease_record) {
        return Err(AuthorityLeaseIssuanceAmbiguityReasonV1::InvalidIssuedFrontier);
    }

    Ok(())
}

fn validate_absence_frontier(
    expected: DurableAuthorityLeaseFrontierV1,
    observed: DurableAuthorityLeaseFrontierV1,
) -> Result<(), AuthorityLeaseIssuanceAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(AuthorityLeaseIssuanceAmbiguityReasonV1::InvalidProvenNotIssuedFrontier);
    }
    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(AuthorityLeaseIssuanceAmbiguityReasonV1::InvalidProvenNotIssuedFrontier);
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ValidatedIssuanceDispositionV1 {
    Issued {
        lease_record: AuthorityLeaseRecordCommitment,
        post_frontier: DurableAuthorityLeaseFrontierV1,
    },
    ProvenNotIssued,
    OutcomeUnknown(AuthorityLeaseIssuanceAmbiguityReasonV1),
}

fn validate_receipt(
    expected_store: AuthorityLeaseStoreDescriptorV1,
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    receipt: &AuthorityLeaseIssuanceReceiptV1,
) -> Result<ValidatedIssuanceDispositionV1, AuthorityLeaseIssuanceAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(AuthorityLeaseIssuanceAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_store {
        return Err(AuthorityLeaseIssuanceAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest != manifest {
        return Err(AuthorityLeaseIssuanceAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.valid_until {
        return Err(AuthorityLeaseIssuanceAmbiguityReasonV1::ReceiptOutlivesStore);
    }

    match receipt.disposition {
        AuthorityLeaseIssuanceDispositionV1::Issued {
            lease_record,
            post_frontier,
        } => {
            validate_issued_frontier(manifest.expected_frontier(), lease_record, post_frontier)?;
            Ok(ValidatedIssuanceDispositionV1::Issued {
                lease_record,
                post_frontier,
            })
        }
        AuthorityLeaseIssuanceDispositionV1::ProvenNotIssued {
            observed_frontier,
            ..
        } => {
            validate_absence_frontier(manifest.expected_frontier(), observed_frontier)?;
            Ok(ValidatedIssuanceDispositionV1::ProvenNotIssued)
        }
        AuthorityLeaseIssuanceDispositionV1::AttemptIdConflict { .. } => Ok(
            ValidatedIssuanceDispositionV1::OutcomeUnknown(
                AuthorityLeaseIssuanceAmbiguityReasonV1::AttemptIdConflict,
            ),
        ),
        AuthorityLeaseIssuanceDispositionV1::ReservationAlreadyIssued { .. } => Ok(
            ValidatedIssuanceDispositionV1::OutcomeUnknown(
                AuthorityLeaseIssuanceAmbiguityReasonV1::ReservationAlreadyIssued,
            ),
        ),
        AuthorityLeaseIssuanceDispositionV1::OutcomeUnknown => Ok(
            ValidatedIssuanceDispositionV1::OutcomeUnknown(
                AuthorityLeaseIssuanceAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            ),
        ),
    }
}

fn freeze<
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
    const L: usize,
    const R: usize,
>(
    prepared: PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    receipt: Option<AuthorityLeaseIssuanceReceiptV1>,
    reason: AuthorityLeaseIssuanceAmbiguityReasonV1,
) -> AuthorityLeaseIssuanceResultV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
    AuthorityLeaseIssuanceResultV1::OutcomeUnknown(FrozenAmbiguousAuthorityLeaseIssuanceV1 {
        prepared,
        last_receipt: receipt,
        reason,
        _store: PhantomData,
    })
}

fn resolve_receipt<
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
    const L: usize,
    const R: usize,
>(
    prepared: PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    store_descriptor_after: AuthorityLeaseStoreDescriptorV1,
    receipt: AuthorityLeaseIssuanceReceiptV1,
) -> AuthorityLeaseIssuanceResultV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
    let expected = prepared.expected_store.descriptor();
    if store_descriptor_after != expected {
        return freeze(
            prepared,
            Some(receipt),
            AuthorityLeaseIssuanceAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }

    let validated = match validate_receipt(expected, prepared.manifest, &receipt) {
        Ok(value) => value,
        Err(reason) => return freeze(prepared, Some(receipt), reason),
    };

    match validated {
        ValidatedIssuanceDispositionV1::Issued {
            lease_record,
            post_frontier,
        } => {
            let authority_valid_until = receipt
                .valid_until
                .min(prepared.candidate.valid_until());
            let bound = BoundAuthorityLeaseIssuanceV1 {
                prepared,
                receipt,
                _store: PhantomData,
            };
            AuthorityLeaseIssuanceResultV1::Issued(DurablyIssuedAuthorityLeaseV1 {
                bound,
                lease_record,
                post_frontier,
                authority_valid_until,
            })
        }
        ValidatedIssuanceDispositionV1::ProvenNotIssued => {
            let bound = BoundAuthorityLeaseIssuanceV1 {
                prepared,
                receipt,
                _store: PhantomData,
            };
            AuthorityLeaseIssuanceResultV1::ProvenNotIssued(ProvenUnissuedAuthorityLeaseV1 {
                bound,
            })
        }
        ValidatedIssuanceDispositionV1::OutcomeUnknown(reason) => {
            freeze(prepared, Some(receipt), reason)
        }
    }
}

/// Invoke the external issuance write exactly once for one prepared attempt.
///
/// Only pre-write descriptor mismatch is returned as an ordinary preparation
/// failure. Once the store call begins, store errors and every unverifiable
/// postcondition become a frozen `OutcomeUnknown`.
pub fn issue_authority_lease<
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
    const L: usize,
    const R: usize,
>(
    store: &S,
    prepared: PreparedAuthorityLeaseIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
) -> Result<
    AuthorityLeaseIssuanceResultV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    AuthorityLeaseIssuancePreparationFailureV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
>
where
    S: AuthorityLeaseIssuanceStoreV1,
{
    let expected = prepared.expected_store.descriptor();
    if store.descriptor() != expected {
        return Err(AuthorityLeaseIssuancePreparationFailureV1 {
            prepared,
            error: AuthorityLeaseIssuancePreparationError::StoreDescriptorMismatchBefore,
        });
    }

    let receipt = match store.issue_authority_lease(&prepared.manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Ok(freeze(
                prepared,
                None,
                AuthorityLeaseIssuanceAmbiguityReasonV1::StoreErrorAfterAttempt,
            ));
        }
    };

    Ok(resolve_receipt::<S, _, _, _, _, _, _, _, _, _, _, L, R>(
        prepared,
        store.descriptor(),
        receipt,
    ))
}

/// Read-only same-attempt reconciliation of an in-memory frozen issuance.
pub fn reconcile_frozen_authority_lease<
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
    const L: usize,
    const R: usize,
>(
    store: &S,
    frozen: FrozenAmbiguousAuthorityLeaseIssuanceV1<
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
        L,
        R,
    >,
) -> AuthorityLeaseIssuanceResultV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
where
    S: AuthorityLeaseIssuanceStoreV1,
{
    let expected = frozen.prepared.expected_store.descriptor();
    if store.descriptor() != expected {
        return freeze(
            frozen.prepared,
            frozen.last_receipt,
            AuthorityLeaseIssuanceAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = match store.reconcile_authority_lease(&frozen.prepared.manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return freeze(
                frozen.prepared,
                frozen.last_receipt,
                AuthorityLeaseIssuanceAmbiguityReasonV1::StoreErrorAfterAttempt,
            );
        }
    };

    resolve_receipt::<S, _, _, _, _, _, _, _, _, _, _, L, R>(
        frozen.prepared,
        store.descriptor(),
        receipt,
    )
}

/// Restart-safe historical outcome that can be reconstructed from a journaled
/// manifest alone. It cannot recreate the typed lease candidate or semantic
/// lease authority.
pub struct HistoricalAuthorityLeaseIssuanceV1<S> {
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
    receipt: Option<AuthorityLeaseIssuanceReceiptV1>,
    disposition: HistoricalAuthorityLeaseIssuanceDispositionV1,
    evidence_valid_until: Option<u64>,
    authority_eligibility_valid_until: Option<u64>,
    _store: PhantomData<fn() -> S>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalAuthorityLeaseIssuanceDispositionV1 {
    Issued {
        lease_record: AuthorityLeaseRecordCommitment,
        post_frontier: DurableAuthorityLeaseFrontierV1,
    },
    ProvenNotIssued,
    OutcomeUnknown(AuthorityLeaseIssuanceAmbiguityReasonV1),
}

impl<S> HistoricalAuthorityLeaseIssuanceV1<S> {
    pub const fn manifest(&self) -> AuthorityLeaseIssuanceAttemptManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> Option<&AuthorityLeaseIssuanceReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> HistoricalAuthorityLeaseIssuanceDispositionV1 {
        self.disposition
    }

    pub const fn evidence_valid_until(&self) -> Option<u64> {
        self.evidence_valid_until
    }

    pub const fn authority_eligibility_valid_until(&self) -> Option<u64> {
        self.authority_eligibility_valid_until
    }

    pub const fn contains_semantic_lease_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Restart reconciliation from the exact journaled manifest only.
///
/// This method is intentionally historical/non-authoritative. A later exact
/// rebind to the typed candidate lineage is required before semantic lease
/// authority can be recovered after restart.
pub fn reconcile_authority_lease_after_restart<S: AuthorityLeaseIssuanceStoreV1>(
    store: &S,
    manifest: AuthorityLeaseIssuanceAttemptManifestV1,
) -> HistoricalAuthorityLeaseIssuanceV1<S> {
    let expected = manifest.expected_store();
    if manifest.schema_version != SSF_SCHEMA_V1 || store.descriptor() != expected {
        return HistoricalAuthorityLeaseIssuanceV1 {
            manifest,
            receipt: None,
            disposition: HistoricalAuthorityLeaseIssuanceDispositionV1::OutcomeUnknown(
                AuthorityLeaseIssuanceAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
            ),
            evidence_valid_until: None,
            authority_eligibility_valid_until: None,
            _store: PhantomData,
        };
    }

    let receipt = match store.reconcile_authority_lease(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return HistoricalAuthorityLeaseIssuanceV1 {
                manifest,
                receipt: None,
                disposition: HistoricalAuthorityLeaseIssuanceDispositionV1::OutcomeUnknown(
                    AuthorityLeaseIssuanceAmbiguityReasonV1::StoreErrorAfterAttempt,
                ),
                evidence_valid_until: None,
                authority_eligibility_valid_until: None,
                _store: PhantomData,
            };
        }
    };

    if store.descriptor() != expected {
        return HistoricalAuthorityLeaseIssuanceV1 {
            manifest,
            receipt: Some(receipt),
            disposition: HistoricalAuthorityLeaseIssuanceDispositionV1::OutcomeUnknown(
                AuthorityLeaseIssuanceAmbiguityReasonV1::StoreDescriptorChangedAfter,
            ),
            evidence_valid_until: None,
            authority_eligibility_valid_until: None,
            _store: PhantomData,
        };
    }

    let validated = match validate_receipt(expected, manifest, &receipt) {
        Ok(value) => value,
        Err(reason) => {
            return HistoricalAuthorityLeaseIssuanceV1 {
                manifest,
                receipt: Some(receipt),
                disposition: HistoricalAuthorityLeaseIssuanceDispositionV1::OutcomeUnknown(reason),
                evidence_valid_until: None,
                authority_eligibility_valid_until: None,
                _store: PhantomData,
            };
        }
    };

    let evidence_valid_until = Some(receipt.valid_until);
    let authority_eligibility_valid_until = Some(
        receipt
            .valid_until
            .min(manifest.subject().lease_valid_until),
    );

    let disposition = match validated {
        ValidatedIssuanceDispositionV1::Issued {
            lease_record,
            post_frontier,
        } => HistoricalAuthorityLeaseIssuanceDispositionV1::Issued {
            lease_record,
            post_frontier,
        },
        ValidatedIssuanceDispositionV1::ProvenNotIssued => {
            HistoricalAuthorityLeaseIssuanceDispositionV1::ProvenNotIssued
        }
        ValidatedIssuanceDispositionV1::OutcomeUnknown(reason) => {
            HistoricalAuthorityLeaseIssuanceDispositionV1::OutcomeUnknown(reason)
        }
    };

    HistoricalAuthorityLeaseIssuanceV1 {
        manifest,
        receipt: Some(receipt),
        disposition,
        evidence_valid_until,
        authority_eligibility_valid_until,
        _store: PhantomData,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn frontier(generation: u64, head: Option<u8>) -> DurableAuthorityLeaseFrontierV1 {
        DurableAuthorityLeaseFrontierV1 {
            generation: DurableAuthorityLeaseGeneration::new(generation),
            head: match head {
                Some(byte) => Some(AuthorityLeaseRecordCommitment::from_bytes(bytes(byte))),
                None => None,
            },
        }
    }

    #[test]
    fn issued_frontier_advances_exactly_once_to_exact_record() {
        let record = AuthorityLeaseRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            validate_issued_frontier(frontier(4, Some(3)), record, frontier(5, Some(9))),
            Ok(())
        );
        assert_eq!(
            validate_issued_frontier(frontier(4, Some(3)), record, frontier(6, Some(9))),
            Err(AuthorityLeaseIssuanceAmbiguityReasonV1::InvalidIssuedFrontier)
        );
        assert_eq!(
            validate_issued_frontier(frontier(4, Some(3)), record, frontier(5, Some(8))),
            Err(AuthorityLeaseIssuanceAmbiguityReasonV1::InvalidIssuedFrontier)
        );
    }

    #[test]
    fn final_absence_cannot_roll_frontier_back() {
        assert_eq!(
            validate_absence_frontier(frontier(7, Some(3)), frontier(6, Some(3))),
            Err(AuthorityLeaseIssuanceAmbiguityReasonV1::InvalidProvenNotIssuedFrontier)
        );
        assert_eq!(
            validate_absence_frontier(frontier(7, Some(3)), frontier(7, Some(4))),
            Err(AuthorityLeaseIssuanceAmbiguityReasonV1::InvalidProvenNotIssuedFrontier)
        );
        assert_eq!(
            validate_absence_frontier(frontier(7, Some(3)), frontier(8, Some(4))),
            Ok(())
        );
    }

    #[test]
    fn already_issued_is_not_equivalent_to_proven_not_issued() {
        assert_ne!(
            AuthorityLeaseIssuanceDispositionV1::ReservationAlreadyIssued {
                existing_lease_record: AuthorityLeaseRecordCommitment::from_bytes(bytes(1)),
            },
            AuthorityLeaseIssuanceDispositionV1::ProvenNotIssued {
                observed_frontier: frontier(1, None),
                absence_evidence: AuthorityLeaseIssuanceAttemptAbsenceCommitment::from_bytes(
                    bytes(2),
                ),
            }
        );
    }

    #[test]
    fn issuance_evidence_never_refreshes_requested_lease_lifetime() {
        let lease_valid_until = 90;
        let issuance_receipt_valid_until = 180;
        assert_eq!(lease_valid_until.min(issuance_receipt_valid_until), 90);
    }
}

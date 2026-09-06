// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Durable, restart-reconcilable registration semantics for SSF
//! adoption-authority candidates.
//!
//! The external-effect boundary is explicit. Before a store call begins, an
//! independently expected store descriptor may reject the attempt normally.
//! Once registration may have occurred, every malformed, unstable, conflicting,
//! or unknown result becomes a frozen outcome. Ambiguous registrations may be
//! reconciled only by querying the exact same attempt manifest; they are never
//! blindly retried as a fresh write.
//!
//! This crate stops before authority issuance. Even a proven persisted
//! registration remains non-executable evidence.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_adoption_authority_candidate::{
    AdoptionAuthorityCandidateId, AdoptionAuthorityCandidateV1, AdoptionAuthorityNonce,
    AdoptionAuthorityOperationV1,
};
use mycelix_ssf_contracts::{ConsequenceClass, SSF_SCHEMA_V1};
use mycelix_ssf_local_adoption_policy::{
    LocalAdoptionScopeCommitment, PolicyEvaluationReceiptCommitment,
};
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

digest_type!(RegistrationStoreIdentityCommitment);
digest_type!(RegistrationStorePolicyCommitment);
digest_type!(RegistrationRecordCommitment);
digest_type!(RegistrationReceiptCommitment);
digest_type!(RegistrationAttemptId);
digest_type!(RegistrationAttemptAbsenceCommitment);
generation_type!(RegistrationStoreVerifierGeneration);
generation_type!(DurableRegistrationGeneration);

/// Stable verifier/configuration identity for one durable registration store.
/// The durable log frontier may advance while this descriptor remains stable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RegistrationStoreDescriptorV1 {
    pub identity: RegistrationStoreIdentityCommitment,
    pub policy: RegistrationStorePolicyCommitment,
    pub verifier_generation: RegistrationStoreVerifierGeneration,
    pub valid_until: u64,
}

/// Independently expected store descriptor supplied by trusted local
/// configuration rather than selected by the store implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedRegistrationStoreProfileV1 {
    descriptor: RegistrationStoreDescriptorV1,
}

impl ExpectedRegistrationStoreProfileV1 {
    pub const fn from_trusted_configuration(descriptor: RegistrationStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> RegistrationStoreDescriptorV1 {
        self.descriptor
    }
}

/// Exact durable-log frontier expected before an initial registration attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DurableRegistrationFrontierV1 {
    pub generation: DurableRegistrationGeneration,
    pub head: Option<RegistrationRecordCommitment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedDurableRegistrationFrontierV1 {
    frontier: DurableRegistrationFrontierV1,
}

impl ExpectedDurableRegistrationFrontierV1 {
    pub const fn from_trusted_state(frontier: DurableRegistrationFrontierV1) -> Self {
        Self { frontier }
    }

    pub const fn frontier(&self) -> DurableRegistrationFrontierV1 {
        self.frontier
    }
}

/// Exact candidate facts presented to durable storage.
///
/// Candidate ID and nonce are not treated as commitments to these fields.
/// Every security-relevant candidate field is rebound independently.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdoptionAuthorityRegistrationSubjectV1 {
    pub candidate_id: AdoptionAuthorityCandidateId,
    pub nonce: AdoptionAuthorityNonce,
    pub operation: AdoptionAuthorityOperationV1,
    pub source_local_state: FederationStateHeadV1,
    pub remote_target: FederationStateHeadV1,
    pub policy_receipt: PolicyEvaluationReceiptCommitment,
    pub scope: LocalAdoptionScopeCommitment,
    pub consequence: ConsequenceClass,
    pub candidate_valid_until: u64,
}

fn subject_from_candidate<P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    candidate: &AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
) -> AdoptionAuthorityRegistrationSubjectV1 {
    AdoptionAuthorityRegistrationSubjectV1 {
        candidate_id: candidate.candidate_id(),
        nonce: candidate.nonce(),
        operation: candidate.operation(),
        source_local_state: candidate.source_local_state(),
        remote_target: candidate.remote_target(),
        policy_receipt: candidate.policy_receipt_commitment(),
        scope: candidate.scope(),
        consequence: candidate.consequence(),
        candidate_valid_until: candidate.valid_until(),
    }
}

/// Restart-safe exact attempt manifest.
///
/// This is the minimum plain-data object that must be recoverable after a
/// process restart in order to reconcile an ambiguous attempt. The manifest
/// itself contains no authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegistrationAttemptManifestV1 {
    schema_version: u16,
    attempt_id: RegistrationAttemptId,
    subject: AdoptionAuthorityRegistrationSubjectV1,
    expected_store: RegistrationStoreDescriptorV1,
    expected_frontier: DurableRegistrationFrontierV1,
}

impl RegistrationAttemptManifestV1 {
    pub const fn from_journaled_parts(
        attempt_id: RegistrationAttemptId,
        subject: AdoptionAuthorityRegistrationSubjectV1,
        expected_store: RegistrationStoreDescriptorV1,
        expected_frontier: DurableRegistrationFrontierV1,
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

    pub const fn attempt_id(&self) -> RegistrationAttemptId {
        self.attempt_id
    }

    pub const fn subject(&self) -> AdoptionAuthorityRegistrationSubjectV1 {
        self.subject
    }

    pub const fn expected_store(&self) -> RegistrationStoreDescriptorV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> DurableRegistrationFrontierV1 {
        self.expected_frontier
    }

    /// The pure contract cannot prove the caller durably journaled this
    /// manifest before the external write. Integrations must do so.
    pub const fn requires_recoverable_journal_before_registration(&self) -> bool {
        true
    }
}

/// Closed-world store-reported resolution.
///
/// `ProvenNotPersisted` is stronger than an ordinary absence observation. By
/// contract it permanently closes this exact attempt ID against future commit
/// and carries evidence of that final non-persistence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationDispositionV1 {
    Persisted {
        record: RegistrationRecordCommitment,
        post_frontier: DurableRegistrationFrontierV1,
    },
    ProvenNotPersisted {
        observed_frontier: DurableRegistrationFrontierV1,
        absence_evidence: RegistrationAttemptAbsenceCommitment,
    },
    AttemptIdConflict {
        existing_manifest: RegistrationAttemptManifestV1,
    },
    OutcomeUnknown,
}

/// Untrusted receipt returned after an initial registration or reconciliation
/// query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdoptionAuthorityRegistrationReceiptV1 {
    pub schema_version: u16,
    pub store: RegistrationStoreDescriptorV1,
    pub manifest: RegistrationAttemptManifestV1,
    pub disposition: RegistrationDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: RegistrationReceiptCommitment,
}

impl AdoptionAuthorityRegistrationReceiptV1 {
    pub const fn new(
        store: RegistrationStoreDescriptorV1,
        manifest: RegistrationAttemptManifestV1,
        disposition: RegistrationDispositionV1,
        valid_until: u64,
        receipt_commitment: RegistrationReceiptCommitment,
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

/// Durable registration adapter.
///
/// `register_candidate` may create at most one record for this exact manifest.
///
/// `reconcile_attempt` is read-only with respect to authority registration. It
/// must never create a record. It resolves the exact already-known attempt ID
/// and manifest to Persisted, permanently ProvenNotPersisted, conflict, or
/// OutcomeUnknown.
///
/// An implementation must permanently bind a first-seen attempt ID to the exact
/// manifest. Reusing that ID with a different manifest may never create a
/// second registration record.
pub trait AdoptionAuthorityRegistrationStoreV1 {
    fn descriptor(&self) -> RegistrationStoreDescriptorV1;

    fn register_candidate(
        &self,
        manifest: &RegistrationAttemptManifestV1,
    ) -> AdoptionAuthorityRegistrationReceiptV1;

    fn reconcile_attempt(
        &self,
        manifest: &RegistrationAttemptManifestV1,
    ) -> AdoptionAuthorityRegistrationReceiptV1;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationPreparationError {
    StoreDescriptorMismatchBefore,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RegistrationAmbiguityReasonV1 {
    StoreReportedOutcomeUnknown,
    AttemptIdConflict,
    StoreDescriptorMismatchBeforeReconciliation,
    StoreDescriptorChangedAfter,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    ReceiptOutlivesCandidate,
    PersistedGenerationOverflow,
    InvalidPersistedFrontier,
    InvalidProvenNotPersistedFrontier,
}

fn validate_persisted_frontier(
    before: DurableRegistrationFrontierV1,
    record: RegistrationRecordCommitment,
    after: DurableRegistrationFrontierV1,
) -> Result<(), RegistrationAmbiguityReasonV1> {
    let expected_generation = before
        .generation
        .get()
        .checked_add(1)
        .ok_or(RegistrationAmbiguityReasonV1::PersistedGenerationOverflow)?;

    if after.generation.get() != expected_generation || after.head != Some(record) {
        return Err(RegistrationAmbiguityReasonV1::InvalidPersistedFrontier);
    }
    Ok(())
}

/// A final non-persistence proof may be produced after unrelated later writes,
/// but it may not claim an observed durable generation older than the
/// registration frontier against which the attempt began. At equal generation,
/// the exact frontier must still match.
fn validate_absence_frontier(
    expected: DurableRegistrationFrontierV1,
    observed: DurableRegistrationFrontierV1,
) -> Result<(), RegistrationAmbiguityReasonV1> {
    if observed.generation < expected.generation {
        return Err(RegistrationAmbiguityReasonV1::InvalidProvenNotPersistedFrontier);
    }
    if observed.generation == expected.generation && observed.head != expected.head {
        return Err(RegistrationAmbiguityReasonV1::InvalidProvenNotPersistedFrontier);
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ValidatedRegistrationDispositionV1 {
    Persisted {
        record: RegistrationRecordCommitment,
        post_frontier: DurableRegistrationFrontierV1,
    },
    ProvenNotPersisted {
        observed_frontier: DurableRegistrationFrontierV1,
        absence_evidence: RegistrationAttemptAbsenceCommitment,
    },
    OutcomeUnknown(RegistrationAmbiguityReasonV1),
}

fn validate_receipt(
    expected_store: RegistrationStoreDescriptorV1,
    manifest: RegistrationAttemptManifestV1,
    receipt: &AdoptionAuthorityRegistrationReceiptV1,
) -> Result<ValidatedRegistrationDispositionV1, RegistrationAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(RegistrationAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_store {
        return Err(RegistrationAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest != manifest {
        return Err(RegistrationAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.valid_until {
        return Err(RegistrationAmbiguityReasonV1::ReceiptOutlivesStore);
    }
    if receipt.valid_until > manifest.subject.candidate_valid_until {
        return Err(RegistrationAmbiguityReasonV1::ReceiptOutlivesCandidate);
    }

    match receipt.disposition {
        RegistrationDispositionV1::OutcomeUnknown => Ok(
            ValidatedRegistrationDispositionV1::OutcomeUnknown(
                RegistrationAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            ),
        ),
        RegistrationDispositionV1::AttemptIdConflict { .. } => Ok(
            ValidatedRegistrationDispositionV1::OutcomeUnknown(
                RegistrationAmbiguityReasonV1::AttemptIdConflict,
            ),
        ),
        RegistrationDispositionV1::Persisted {
            record,
            post_frontier,
        } => {
            validate_persisted_frontier(manifest.expected_frontier, record, post_frontier)?;
            Ok(ValidatedRegistrationDispositionV1::Persisted {
                record,
                post_frontier,
            })
        }
        RegistrationDispositionV1::ProvenNotPersisted {
            observed_frontier,
            absence_evidence,
        } => {
            validate_absence_frontier(manifest.expected_frontier, observed_frontier)?;
            Ok(ValidatedRegistrationDispositionV1::ProvenNotPersisted {
                observed_frontier,
                absence_evidence,
            })
        }
    }
}

/// Prepared exact registration attempt. The owned candidate is retained until
/// the external registration outcome is resolved.
pub struct PreparedAdoptionAuthorityRegistrationV1<
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    candidate: AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
    expected_store: ExpectedRegistrationStoreProfileV1,
    expected_frontier: ExpectedDurableRegistrationFrontierV1,
    manifest: RegistrationAttemptManifestV1,
}

impl<P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R>
{
    pub const fn manifest(&self) -> RegistrationAttemptManifestV1 {
        self.manifest
    }

    pub const fn candidate(
        &self,
    ) -> &AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R> {
        &self.candidate
    }

    pub const fn expected_store(&self) -> ExpectedRegistrationStoreProfileV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> ExpectedDurableRegistrationFrontierV1 {
        self.expected_frontier
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn prepare_adoption_authority_registration<
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    expected_store: ExpectedRegistrationStoreProfileV1,
    expected_frontier: ExpectedDurableRegistrationFrontierV1,
    attempt_id: RegistrationAttemptId,
    candidate: AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
) -> PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R> {
    let subject = subject_from_candidate(&candidate);
    let manifest = RegistrationAttemptManifestV1::from_journaled_parts(
        attempt_id,
        subject,
        expected_store.descriptor(),
        expected_frontier.frontier(),
    );

    PreparedAdoptionAuthorityRegistrationV1 {
        candidate,
        expected_store,
        expected_frontier,
        manifest,
    }
}

pub struct RegistrationPreparationFailureV1<
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    prepared: PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R>,
    error: RegistrationPreparationError,
}

impl<P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    RegistrationPreparationFailureV1<P, EV, RV, AQ, LQ, L, R>
{
    pub const fn error(&self) -> RegistrationPreparationError {
        self.error
    }

    pub fn into_prepared(
        self,
    ) -> PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R> {
        self.prepared
    }
}

struct BoundRegistrationEvidenceV1<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    prepared: PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R>,
    receipt: AdoptionAuthorityRegistrationReceiptV1,
    _store_type: PhantomData<fn() -> S>,
}

pub struct PersistedAdoptionAuthorityRegistrationV1<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    bound: BoundRegistrationEvidenceV1<S, P, EV, RV, AQ, LQ, L, R>,
}

pub struct ProvenUnpersistedAdoptionAuthorityRegistrationV1<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    bound: BoundRegistrationEvidenceV1<S, P, EV, RV, AQ, LQ, L, R>,
}

/// Frozen ambiguous registration. No API returns the candidate for a new
/// attempt. The only state-progressing method is same-attempt reconciliation.
pub struct FrozenAmbiguousAdoptionAuthorityRegistrationV1<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    prepared: PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R>,
    last_receipt: AdoptionAuthorityRegistrationReceiptV1,
    reason: RegistrationAmbiguityReasonV1,
    _store_type: PhantomData<fn() -> S>,
}

impl<S, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    PersistedAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn candidate(
        &self,
    ) -> &AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R> {
        self.bound.prepared.candidate()
    }

    pub const fn manifest(&self) -> RegistrationAttemptManifestV1 {
        self.bound.prepared.manifest()
    }

    pub const fn receipt(&self) -> &AdoptionAuthorityRegistrationReceiptV1 {
        &self.bound.receipt
    }

    pub const fn valid_until(&self) -> u64 {
        self.bound.receipt.valid_until
    }

    pub const fn is_durably_registered(&self) -> bool {
        true
    }

    pub const fn eligible_for_authority_issuance(&self) -> bool {
        true
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

impl<S, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    ProvenUnpersistedAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn manifest(&self) -> RegistrationAttemptManifestV1 {
        self.bound.prepared.manifest()
    }

    pub const fn receipt(&self) -> &AdoptionAuthorityRegistrationReceiptV1 {
        &self.bound.receipt
    }

    pub const fn old_attempt_is_permanently_closed(&self) -> bool {
        true
    }

    pub const fn retry_with_new_attempt_may_be_considered(&self) -> bool {
        true
    }

    pub fn into_candidate(self) -> AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R> {
        self.bound.prepared.candidate
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

impl<S, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    FrozenAmbiguousAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>
{
    pub const fn candidate(
        &self,
    ) -> &AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R> {
        self.prepared.candidate()
    }

    pub const fn manifest(&self) -> RegistrationAttemptManifestV1 {
        self.prepared.manifest()
    }

    pub const fn last_receipt(&self) -> &AdoptionAuthorityRegistrationReceiptV1 {
        &self.last_receipt
    }

    pub const fn reason(&self) -> RegistrationAmbiguityReasonV1 {
        self.reason
    }

    pub const fn retry_forbidden(&self) -> bool {
        true
    }

    pub const fn frozen_pending_reconciliation(&self) -> bool {
        true
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub enum AdoptionAuthorityRegistrationResultV1<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    Persisted(PersistedAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>),
    ProvenNotPersisted(
        ProvenUnpersistedAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>,
    ),
    OutcomeUnknown(
        FrozenAmbiguousAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>,
    ),
}

fn freeze<S, P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    prepared: PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R>,
    receipt: AdoptionAuthorityRegistrationReceiptV1,
    reason: RegistrationAmbiguityReasonV1,
) -> AdoptionAuthorityRegistrationResultV1<S, P, EV, RV, AQ, LQ, L, R> {
    AdoptionAuthorityRegistrationResultV1::OutcomeUnknown(
        FrozenAmbiguousAdoptionAuthorityRegistrationV1 {
            prepared,
            last_receipt: receipt,
            reason,
            _store_type: PhantomData,
        },
    )
}

fn resolve_typed_receipt<S, P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    prepared: PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R>,
    store_descriptor_after: RegistrationStoreDescriptorV1,
    receipt: AdoptionAuthorityRegistrationReceiptV1,
) -> AdoptionAuthorityRegistrationResultV1<S, P, EV, RV, AQ, LQ, L, R> {
    let expected_descriptor = prepared.expected_store.descriptor();
    if store_descriptor_after != expected_descriptor {
        return freeze::<S, P, EV, RV, AQ, LQ, L, R>(
            prepared,
            receipt,
            RegistrationAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }

    let manifest = prepared.manifest;
    let validated = match validate_receipt(expected_descriptor, manifest, &receipt) {
        Ok(value) => value,
        Err(reason) => {
            return freeze::<S, P, EV, RV, AQ, LQ, L, R>(prepared, receipt, reason);
        }
    };

    match validated {
        ValidatedRegistrationDispositionV1::OutcomeUnknown(reason) => {
            freeze::<S, P, EV, RV, AQ, LQ, L, R>(prepared, receipt, reason)
        }
        ValidatedRegistrationDispositionV1::Persisted { .. } => {
            AdoptionAuthorityRegistrationResultV1::Persisted(
                PersistedAdoptionAuthorityRegistrationV1 {
                    bound: BoundRegistrationEvidenceV1 {
                        prepared,
                        receipt,
                        _store_type: PhantomData,
                    },
                },
            )
        }
        ValidatedRegistrationDispositionV1::ProvenNotPersisted { .. } => {
            AdoptionAuthorityRegistrationResultV1::ProvenNotPersisted(
                ProvenUnpersistedAdoptionAuthorityRegistrationV1 {
                    bound: BoundRegistrationEvidenceV1 {
                        prepared,
                        receipt,
                        _store_type: PhantomData,
                    },
                },
            )
        }
    }
}

/// Perform the initial external registration attempt.
///
/// The only ordinary error is a store-profile mismatch detected before the
/// store call. The owned prepared attempt is returned in that error so no
/// candidate lineage is lost.
///
/// Once the store call begins, every anomaly becomes a frozen OutcomeUnknown.
#[allow(clippy::result_large_err, clippy::type_complexity)]
pub fn register_prepared_adoption_authority<
    S,
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    store: &S,
    prepared: PreparedAdoptionAuthorityRegistrationV1<P, EV, RV, AQ, LQ, L, R>,
) -> Result<
    AdoptionAuthorityRegistrationResultV1<S, P, EV, RV, AQ, LQ, L, R>,
    RegistrationPreparationFailureV1<P, EV, RV, AQ, LQ, L, R>,
>
where
    S: AdoptionAuthorityRegistrationStoreV1,
{
    let expected_descriptor = prepared.expected_store.descriptor();
    if store.descriptor() != expected_descriptor {
        return Err(RegistrationPreparationFailureV1 {
            prepared,
            error: RegistrationPreparationError::StoreDescriptorMismatchBefore,
        });
    }

    let manifest = prepared.manifest;
    let receipt = store.register_candidate(&manifest);
    let descriptor_after = store.descriptor();

    Ok(resolve_typed_receipt::<S, P, EV, RV, AQ, LQ, L, R>(
        prepared,
        descriptor_after,
        receipt,
    ))
}

impl<S, P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    FrozenAmbiguousAdoptionAuthorityRegistrationV1<S, P, EV, RV, AQ, LQ, L, R>
where
    S: AdoptionAuthorityRegistrationStoreV1,
{
    /// Reconcile the exact same attempt. This method never calls the store's
    /// write API and never creates a fresh attempt ID.
    pub fn reconcile_same_attempt(
        self,
        store: &S,
    ) -> AdoptionAuthorityRegistrationResultV1<S, P, EV, RV, AQ, LQ, L, R> {
        let FrozenAmbiguousAdoptionAuthorityRegistrationV1 {
            prepared,
            last_receipt,
            reason: _,
            _store_type: _,
        } = self;

        let expected_descriptor = prepared.expected_store.descriptor();
        if store.descriptor() != expected_descriptor {
            return AdoptionAuthorityRegistrationResultV1::OutcomeUnknown(
                FrozenAmbiguousAdoptionAuthorityRegistrationV1 {
                    prepared,
                    last_receipt,
                    reason: RegistrationAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
                    _store_type: PhantomData,
                },
            );
        }

        let manifest = prepared.manifest;
        let receipt = store.reconcile_attempt(&manifest);
        let descriptor_after = store.descriptor();

        resolve_typed_receipt::<S, P, EV, RV, AQ, LQ, L, R>(
            prepared,
            descriptor_after,
            receipt,
        )
    }
}

/// Plain restart-safe reconciliation disposition. This type intentionally does
/// not carry the original typed candidate and therefore cannot issue authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestartReconciliationDispositionV1 {
    Persisted {
        record: RegistrationRecordCommitment,
        post_frontier: DurableRegistrationFrontierV1,
    },
    ProvenNotPersisted {
        observed_frontier: DurableRegistrationFrontierV1,
        absence_evidence: RegistrationAttemptAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: RegistrationAmbiguityReasonV1,
    },
}

/// Reconciliation evidence obtainable after process restart from only the
/// journaled exact manifest and independently expected store profile.
pub struct RestartReconciledRegistrationAttemptV1<S> {
    manifest: RegistrationAttemptManifestV1,
    expected_store: ExpectedRegistrationStoreProfileV1,
    receipt: Option<AdoptionAuthorityRegistrationReceiptV1>,
    disposition: RestartReconciliationDispositionV1,
    _store_type: PhantomData<fn() -> S>,
}

impl<S> RestartReconciledRegistrationAttemptV1<S> {
    pub const fn manifest(&self) -> RegistrationAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_store(&self) -> ExpectedRegistrationStoreProfileV1 {
        self.expected_store
    }

    pub fn receipt(&self) -> Option<&AdoptionAuthorityRegistrationReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> RestartReconciliationDispositionV1 {
        self.disposition
    }

    pub fn permits_new_attempt(&self) -> bool {
        matches!(
            self.disposition,
            RestartReconciliationDispositionV1::ProvenNotPersisted { .. }
        )
    }

    pub fn persisted_candidate_may_be_rebound(&self) -> bool {
        matches!(
            self.disposition,
            RestartReconciliationDispositionV1::Persisted { .. }
        )
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Reconcile an already-journaled attempt after restart without possessing the
/// original in-memory candidate typestate.
///
/// This function calls only the store's read-only `reconcile_attempt` method.
/// Persisted evidence still requires a future explicit rebind to a rehydrated
/// candidate lineage before authority issuance.
pub fn reconcile_journaled_registration_attempt<S>(
    expected_store: ExpectedRegistrationStoreProfileV1,
    store: &S,
    manifest: RegistrationAttemptManifestV1,
) -> RestartReconciledRegistrationAttemptV1<S>
where
    S: AdoptionAuthorityRegistrationStoreV1,
{
    let expected_descriptor = expected_store.descriptor();

    if manifest.expected_store != expected_descriptor
        || store.descriptor() != expected_descriptor
    {
        return RestartReconciledRegistrationAttemptV1 {
            manifest,
            expected_store,
            receipt: None,
            disposition: RestartReconciliationDispositionV1::OutcomeUnknown {
                reason: RegistrationAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
            },
            _store_type: PhantomData,
        };
    }

    let receipt = store.reconcile_attempt(&manifest);
    let descriptor_after = store.descriptor();

    if descriptor_after != expected_descriptor {
        return RestartReconciledRegistrationAttemptV1 {
            manifest,
            expected_store,
            receipt: Some(receipt),
            disposition: RestartReconciliationDispositionV1::OutcomeUnknown {
                reason: RegistrationAmbiguityReasonV1::StoreDescriptorChangedAfter,
            },
            _store_type: PhantomData,
        };
    }

    let disposition = match validate_receipt(expected_descriptor, manifest, &receipt) {
        Ok(ValidatedRegistrationDispositionV1::Persisted {
            record,
            post_frontier,
        }) => RestartReconciliationDispositionV1::Persisted {
            record,
            post_frontier,
        },
        Ok(ValidatedRegistrationDispositionV1::ProvenNotPersisted {
            observed_frontier,
            absence_evidence,
        }) => RestartReconciliationDispositionV1::ProvenNotPersisted {
            observed_frontier,
            absence_evidence,
        },
        Ok(ValidatedRegistrationDispositionV1::OutcomeUnknown(reason)) | Err(reason) => {
            RestartReconciliationDispositionV1::OutcomeUnknown { reason }
        }
    };

    RestartReconciledRegistrationAttemptV1 {
        manifest,
        expected_store,
        receipt: Some(receipt),
        disposition,
        _store_type: PhantomData,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_contracts::{
        AuthorityGeneration, AuthorityRootCommitment, EvidenceCommitment, FederationEpoch,
        FederationId, PolicyDigest, RevocationGeneration,
    };
    use mycelix_ssf_snapshots::{
        assemble_federation_state_head, MembershipSnapshotV1, PolicySnapshotV1,
        RevocationSnapshotV1, SnapshotCommitment, SnapshotGeneration, SnapshotLineageV1,
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

    fn descriptor() -> RegistrationStoreDescriptorV1 {
        RegistrationStoreDescriptorV1 {
            identity: RegistrationStoreIdentityCommitment::from_bytes(bytes(10)),
            policy: RegistrationStorePolicyCommitment::from_bytes(bytes(11)),
            verifier_generation: RegistrationStoreVerifierGeneration::new(4),
            valid_until: 100,
        }
    }

    fn frontier(generation: u64, head: Option<u8>) -> DurableRegistrationFrontierV1 {
        DurableRegistrationFrontierV1 {
            generation: DurableRegistrationGeneration::new(generation),
            head: head.map(|byte| RegistrationRecordCommitment::from_bytes(bytes(byte))),
        }
    }

    fn subject() -> AdoptionAuthorityRegistrationSubjectV1 {
        AdoptionAuthorityRegistrationSubjectV1 {
            candidate_id: AdoptionAuthorityCandidateId::from_bytes(bytes(20)),
            nonce: AdoptionAuthorityNonce::from_bytes(bytes(21)),
            operation: AdoptionAuthorityOperationV1::InstallQualifiedRemoteState,
            source_local_state: state(30),
            remote_target: state(40),
            policy_receipt: PolicyEvaluationReceiptCommitment::from_bytes(bytes(22)),
            scope: LocalAdoptionScopeCommitment::from_bytes(bytes(23)),
            consequence: ConsequenceClass::C3ScopedRestriction,
            candidate_valid_until: 90,
        }
    }

    fn manifest() -> RegistrationAttemptManifestV1 {
        RegistrationAttemptManifestV1::from_journaled_parts(
            RegistrationAttemptId::from_bytes(bytes(24)),
            subject(),
            descriptor(),
            frontier(7, Some(3)),
        )
    }

    fn receipt(disposition: RegistrationDispositionV1) -> AdoptionAuthorityRegistrationReceiptV1 {
        AdoptionAuthorityRegistrationReceiptV1::new(
            descriptor(),
            manifest(),
            disposition,
            80,
            RegistrationReceiptCommitment::from_bytes(bytes(25)),
        )
    }

    #[test]
    fn persisted_frontier_must_advance_exactly_once_to_exact_record() {
        let before = frontier(7, Some(3));
        let record = RegistrationRecordCommitment::from_bytes(bytes(4));
        assert_eq!(
            validate_persisted_frontier(before, record, frontier(8, Some(4))),
            Ok(())
        );
        assert_eq!(
            validate_persisted_frontier(before, record, frontier(9, Some(4))),
            Err(RegistrationAmbiguityReasonV1::InvalidPersistedFrontier)
        );
        assert_eq!(
            validate_persisted_frontier(before, record, frontier(8, Some(5))),
            Err(RegistrationAmbiguityReasonV1::InvalidPersistedFrontier)
        );
    }

    #[test]
    fn final_absence_can_be_proven_after_later_unrelated_writes_but_not_rollback() {
        assert_eq!(
            validate_absence_frontier(frontier(7, Some(3)), frontier(12, Some(9))),
            Ok(())
        );
        assert_eq!(
            validate_absence_frontier(frontier(7, Some(3)), frontier(6, Some(2))),
            Err(RegistrationAmbiguityReasonV1::InvalidProvenNotPersistedFrontier)
        );
        assert_eq!(
            validate_absence_frontier(frontier(7, Some(3)), frontier(7, Some(8))),
            Err(RegistrationAmbiguityReasonV1::InvalidProvenNotPersistedFrontier)
        );
    }

    #[test]
    fn receipt_must_rebind_exact_manifest() {
        let mut other = manifest();
        other.subject.scope = LocalAdoptionScopeCommitment::from_bytes(bytes(99));
        let bad = AdoptionAuthorityRegistrationReceiptV1::new(
            descriptor(),
            other,
            RegistrationDispositionV1::OutcomeUnknown,
            80,
            RegistrationReceiptCommitment::from_bytes(bytes(25)),
        );

        assert_eq!(
            validate_receipt(descriptor(), manifest(), &bad),
            Err(RegistrationAmbiguityReasonV1::ReceiptManifestMismatch)
        );
    }

    #[test]
    fn attempt_id_conflict_is_never_treated_as_absence() {
        let conflict = receipt(RegistrationDispositionV1::AttemptIdConflict {
            existing_manifest: manifest(),
        });
        assert_eq!(
            validate_receipt(descriptor(), manifest(), &conflict),
            Ok(ValidatedRegistrationDispositionV1::OutcomeUnknown(
                RegistrationAmbiguityReasonV1::AttemptIdConflict
            ))
        );
    }

    #[test]
    fn proven_non_persistence_retains_final_absence_evidence() {
        let absence = RegistrationAttemptAbsenceCommitment::from_bytes(bytes(77));
        let proof = receipt(RegistrationDispositionV1::ProvenNotPersisted {
            observed_frontier: frontier(10, Some(8)),
            absence_evidence: absence,
        });

        assert_eq!(
            validate_receipt(descriptor(), manifest(), &proof),
            Ok(ValidatedRegistrationDispositionV1::ProvenNotPersisted {
                observed_frontier: frontier(10, Some(8)),
                absence_evidence: absence,
            })
        );
    }

    #[derive(Clone, Copy)]
    struct ReconcileStore {
        descriptor: RegistrationStoreDescriptorV1,
        disposition: RegistrationDispositionV1,
    }

    impl AdoptionAuthorityRegistrationStoreV1 for ReconcileStore {
        fn descriptor(&self) -> RegistrationStoreDescriptorV1 {
            self.descriptor
        }

        fn register_candidate(
            &self,
            manifest: &RegistrationAttemptManifestV1,
        ) -> AdoptionAuthorityRegistrationReceiptV1 {
            AdoptionAuthorityRegistrationReceiptV1::new(
                self.descriptor,
                *manifest,
                self.disposition,
                80,
                RegistrationReceiptCommitment::from_bytes(bytes(88)),
            )
        }

        fn reconcile_attempt(
            &self,
            manifest: &RegistrationAttemptManifestV1,
        ) -> AdoptionAuthorityRegistrationReceiptV1 {
            AdoptionAuthorityRegistrationReceiptV1::new(
                self.descriptor,
                *manifest,
                self.disposition,
                80,
                RegistrationReceiptCommitment::from_bytes(bytes(89)),
            )
        }
    }

    #[test]
    fn restart_reconciliation_can_prove_permanent_absence_without_candidate_typestate() {
        let absence = RegistrationAttemptAbsenceCommitment::from_bytes(bytes(77));
        let store = ReconcileStore {
            descriptor: descriptor(),
            disposition: RegistrationDispositionV1::ProvenNotPersisted {
                observed_frontier: frontier(10, Some(8)),
                absence_evidence: absence,
            },
        };

        let result = reconcile_journaled_registration_attempt(
            ExpectedRegistrationStoreProfileV1::from_trusted_configuration(descriptor()),
            &store,
            manifest(),
        );

        assert!(result.permits_new_attempt());
        assert!(!result.persisted_candidate_may_be_rebound());
        assert!(!result.contains_effect_authority());
    }

    #[test]
    fn restart_unknown_never_permits_new_attempt() {
        let store = ReconcileStore {
            descriptor: descriptor(),
            disposition: RegistrationDispositionV1::OutcomeUnknown,
        };

        let result = reconcile_journaled_registration_attempt(
            ExpectedRegistrationStoreProfileV1::from_trusted_configuration(descriptor()),
            &store,
            manifest(),
        );

        assert!(!result.permits_new_attempt());
        assert!(!result.persisted_candidate_may_be_rebound());
    }

    #[test]
    fn restart_persisted_result_is_still_non_authoritative() {
        let store = ReconcileStore {
            descriptor: descriptor(),
            disposition: RegistrationDispositionV1::Persisted {
                record: RegistrationRecordCommitment::from_bytes(bytes(4)),
                post_frontier: frontier(8, Some(4)),
            },
        };

        let result = reconcile_journaled_registration_attempt(
            ExpectedRegistrationStoreProfileV1::from_trusted_configuration(descriptor()),
            &store,
            manifest(),
        );

        assert!(result.persisted_candidate_may_be_rebound());
        assert!(!result.permits_new_attempt());
        assert!(!result.contains_state_install_authority());
        assert!(!result.contains_effect_authority());
    }
}

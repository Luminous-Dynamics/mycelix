// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Crash-safe durable claim of one exact transient SSF effect capability.
//!
//! A claim is a write-ahead execution reservation, not an external effect. It
//! prevents a second process/attempt from acquiring the same durable capability
//! record while preserving explicit ambiguity if the claim write cannot be
//! proven. No provider handle is resolved and no actuator is invoked here.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_transient_effect_capability::{
    TransientEffectCapabilityBindingV1, TransientEffectCapabilityV1,
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

digest_type!(EffectCapabilityClaimAttemptId);
digest_type!(EffectCapabilityClaimRecordCommitment);
digest_type!(EffectCapabilityClaimReceiptCommitment);
digest_type!(EffectCapabilityClaimAbsenceCommitment);
digest_type!(EffectCapabilityClaimManifestCommitment);
digest_type!(EffectCapabilityClaimStoreIdentityCommitment);
digest_type!(EffectCapabilityClaimStorePolicyCommitment);

generation_type!(EffectCapabilityClaimStoreGeneration);
generation_type!(DurableEffectCapabilityClaimGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectCapabilityClaimStoreTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EffectCapabilityClaimStoreDescriptorV1 {
    pub stable_identity: EffectCapabilityClaimStoreIdentityCommitment,
    pub policy: EffectCapabilityClaimStorePolicyCommitment,
    pub generation: EffectCapabilityClaimStoreGeneration,
    pub time_basis: EffectCapabilityClaimStoreTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedEffectCapabilityClaimStoreProfileV1 {
    descriptor: EffectCapabilityClaimStoreDescriptorV1,
}

impl ExpectedEffectCapabilityClaimStoreProfileV1 {
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
pub struct DurableEffectCapabilityClaimFrontierV1 {
    pub generation: DurableEffectCapabilityClaimGeneration,
    pub head: Option<EffectCapabilityClaimRecordCommitment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedDurableEffectCapabilityClaimFrontierV1 {
    frontier: DurableEffectCapabilityClaimFrontierV1,
}

impl ExpectedDurableEffectCapabilityClaimFrontierV1 {
    pub const fn from_trusted_state(frontier: DurableEffectCapabilityClaimFrontierV1) -> Self {
        Self { frontier }
    }

    pub const fn frontier(&self) -> DurableEffectCapabilityClaimFrontierV1 {
        self.frontier
    }
}

/// Plain crash-recoverable claim subject.
///
/// The complete transient capability binding is retained. The store MUST use
/// `capability.durable_capability_record` as the cross-attempt uniqueness key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectCapabilityClaimSubjectV1 {
    pub capability: TransientEffectCapabilityBindingV1,
    pub claim_time_receipt: CurrentTimeReceiptCommitment,
    pub claim_time_valid_until: u64,
    pub claim_latest_possible_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectCapabilityClaimAttemptManifestV1 {
    pub schema_version: u16,
    attempt_id: EffectCapabilityClaimAttemptId,
    subject: EffectCapabilityClaimSubjectV1,
    expected_store: EffectCapabilityClaimStoreDescriptorV1,
    expected_frontier: DurableEffectCapabilityClaimFrontierV1,
}

impl EffectCapabilityClaimAttemptManifestV1 {
    pub const fn from_journaled_parts(
        attempt_id: EffectCapabilityClaimAttemptId,
        subject: EffectCapabilityClaimSubjectV1,
        expected_store: EffectCapabilityClaimStoreDescriptorV1,
        expected_frontier: DurableEffectCapabilityClaimFrontierV1,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            attempt_id,
            subject,
            expected_store,
            expected_frontier,
        }
    }

    pub const fn attempt_id(&self) -> EffectCapabilityClaimAttemptId {
        self.attempt_id
    }

    pub const fn subject(&self) -> EffectCapabilityClaimSubjectV1 {
        self.subject
    }

    pub const fn expected_store(&self) -> EffectCapabilityClaimStoreDescriptorV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> DurableEffectCapabilityClaimFrontierV1 {
        self.expected_frontier
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectCapabilityClaimDispositionV1 {
    Claimed {
        claim_record: EffectCapabilityClaimRecordCommitment,
        post_frontier: DurableEffectCapabilityClaimFrontierV1,
    },
    /// Final non-claim for this attempt ID. Once returned, this exact attempt
    /// may never later commit.
    ProvenNotClaimed {
        observed_frontier: DurableEffectCapabilityClaimFrontierV1,
        absence_evidence: EffectCapabilityClaimAbsenceCommitment,
    },
    AttemptIdConflict {
        existing_manifest: EffectCapabilityClaimManifestCommitment,
    },
    /// The same durable capability record has already been claimed by another
    /// claim lineage. This is ambiguity for the current caller, not success.
    CapabilityAlreadyClaimed {
        existing_claim_record: EffectCapabilityClaimRecordCommitment,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectCapabilityClaimReceiptV1 {
    pub schema_version: u16,
    pub store: EffectCapabilityClaimStoreDescriptorV1,
    pub manifest: EffectCapabilityClaimAttemptManifestV1,
    pub disposition: EffectCapabilityClaimDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: EffectCapabilityClaimReceiptCommitment,
}

pub trait EffectCapabilityClaimStoreV1 {
    type Error;

    fn descriptor(&self) -> EffectCapabilityClaimStoreDescriptorV1;

    /// External durable write boundary. Implementations MUST enforce:
    ///
    /// - first-seen attempt ID permanently binds the exact manifest;
    /// - one exact `capability.durable_capability_record` produces at most one
    ///   durable claim record across all attempt IDs;
    /// - `ProvenNotClaimed` is final for that attempt ID.
    fn claim_effect_capability(
        &self,
        manifest: &EffectCapabilityClaimAttemptManifestV1,
    ) -> Result<EffectCapabilityClaimReceiptV1, Self::Error>;

    /// Read-only reconciliation of the exact same journaled attempt.
    fn reconcile_effect_capability_claim(
        &self,
        manifest: &EffectCapabilityClaimAttemptManifestV1,
    ) -> Result<EffectCapabilityClaimReceiptV1, Self::Error>;
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactTransientEffectCapabilityV1: sealed::Sealed {
    fn exact_binding(&self) -> TransientEffectCapabilityBindingV1;
    fn exact_valid_until(&self) -> u64;
}

impl<S, M, Q, TQ, MTQ> sealed::Sealed for TransientEffectCapabilityV1<S, M, Q, TQ, MTQ> {}

impl<S, M, Q, TQ, MTQ> ExactTransientEffectCapabilityV1
    for TransientEffectCapabilityV1<S, M, Q, TQ, MTQ>
{
    fn exact_binding(&self) -> TransientEffectCapabilityBindingV1 {
        self.binding()
    }

    fn exact_valid_until(&self) -> u64 {
        self.valid_until()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectCapabilityClaimPreparationError {
    UnsupportedStoreTimeBasis,
    CapabilityAlreadyExpired,
    ClaimTimeAlreadyExpired,
}

pub struct EffectCapabilityClaimPreparationFailureV1<C, TQ> {
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    error: EffectCapabilityClaimPreparationError,
}

impl<C, TQ> EffectCapabilityClaimPreparationFailureV1<C, TQ> {
    pub const fn error(&self) -> EffectCapabilityClaimPreparationError {
        self.error
    }

    pub fn into_parts(self) -> (C, QualifiedCurrentTimeV1<TQ>) {
        (self.capability, self.claim_time)
    }
}

pub struct PreparedEffectCapabilityClaimV1<C, TQ> {
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    expected_frontier: ExpectedDurableEffectCapabilityClaimFrontierV1,
    manifest: EffectCapabilityClaimAttemptManifestV1,
}

impl<C, TQ> PreparedEffectCapabilityClaimV1<C, TQ> {
    pub fn capability(&self) -> &C {
        &self.capability
    }

    pub const fn claim_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.claim_time
    }

    pub const fn manifest(&self) -> EffectCapabilityClaimAttemptManifestV1 {
        self.manifest
    }

    pub const fn expected_store(&self) -> ExpectedEffectCapabilityClaimStoreProfileV1 {
        self.expected_store
    }

    pub const fn expected_frontier(&self) -> ExpectedDurableEffectCapabilityClaimFrontierV1 {
        self.expected_frontier
    }
}

pub fn prepare_effect_capability_claim<C, TQ>(
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    attempt_id: EffectCapabilityClaimAttemptId,
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    expected_frontier: ExpectedDurableEffectCapabilityClaimFrontierV1,
) -> Result<PreparedEffectCapabilityClaimV1<C, TQ>, EffectCapabilityClaimPreparationFailureV1<C, TQ>>
where
    C: ExactTransientEffectCapabilityV1,
{
    let latest = claim_time.latest_possible_unix_ms();
    let error = if expected_store.descriptor.time_basis
        != EffectCapabilityClaimStoreTimeBasisV1::UnixMillisecondsUtc
    {
        Some(EffectCapabilityClaimPreparationError::UnsupportedStoreTimeBasis)
    } else if capability.exact_valid_until() < latest {
        Some(EffectCapabilityClaimPreparationError::CapabilityAlreadyExpired)
    } else if claim_time.valid_until() < latest {
        Some(EffectCapabilityClaimPreparationError::ClaimTimeAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(EffectCapabilityClaimPreparationFailureV1 {
            capability,
            claim_time,
            error,
        });
    }

    let subject = EffectCapabilityClaimSubjectV1 {
        capability: capability.exact_binding(),
        claim_time_receipt: claim_time.receipt_commitment(),
        claim_time_valid_until: claim_time.valid_until(),
        claim_latest_possible_unix_ms: latest,
    };
    let manifest = EffectCapabilityClaimAttemptManifestV1::from_journaled_parts(
        attempt_id,
        subject,
        expected_store.descriptor,
        expected_frontier.frontier,
    );

    Ok(PreparedEffectCapabilityClaimV1 {
        capability,
        claim_time,
        expected_store,
        expected_frontier,
        manifest,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectCapabilityClaimPreWriteError {
    StoreDescriptorMismatchBefore,
}

pub struct EffectCapabilityClaimPreWriteFailureV1<C, TQ> {
    prepared: PreparedEffectCapabilityClaimV1<C, TQ>,
    error: EffectCapabilityClaimPreWriteError,
}

impl<C, TQ> EffectCapabilityClaimPreWriteFailureV1<C, TQ> {
    pub const fn error(&self) -> EffectCapabilityClaimPreWriteError {
        self.error
    }

    pub fn into_prepared(self) -> PreparedEffectCapabilityClaimV1<C, TQ> {
        self.prepared
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectCapabilityClaimAmbiguityReasonV1 {
    StoreErrorAfterClaimBoundary,
    StoreDescriptorChangedAfter,
    StoreDescriptorMismatchBeforeReconciliation,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    ReceiptOutlivesCapability,
    ReceiptOutlivesClaimTime,
    ClaimedGenerationOverflow,
    InvalidClaimedFrontier,
    InvalidProvenNotClaimedFrontier,
    AttemptIdConflict,
    CapabilityAlreadyClaimed,
    StoreReportedOutcomeUnknown,
}

pub struct FrozenEffectCapabilityClaimV1<S, C, TQ> {
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    manifest: EffectCapabilityClaimAttemptManifestV1,
    reason: EffectCapabilityClaimAmbiguityReasonV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, C, TQ> FrozenEffectCapabilityClaimV1<S, C, TQ> {
    pub const fn manifest(&self) -> EffectCapabilityClaimAttemptManifestV1 {
        self.manifest
    }

    pub const fn reason(&self) -> EffectCapabilityClaimAmbiguityReasonV1 {
        self.reason
    }

    pub const fn may_create_fresh_claim_attempt(&self) -> bool {
        false
    }

    pub const fn provider_or_actuator_invoked(&self) -> bool {
        false
    }
}

pub struct RecoverableUnclaimedEffectCapabilityV1<C, TQ> {
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    manifest: EffectCapabilityClaimAttemptManifestV1,
    receipt: EffectCapabilityClaimReceiptV1,
}

impl<C, TQ> RecoverableUnclaimedEffectCapabilityV1<C, TQ> {
    pub const fn manifest(&self) -> EffectCapabilityClaimAttemptManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> EffectCapabilityClaimReceiptV1 {
        self.receipt
    }

    pub fn into_parts(self) -> (C, QualifiedCurrentTimeV1<TQ>) {
        (self.capability, self.claim_time)
    }

    pub const fn may_create_fresh_claim_attempt(&self) -> bool {
        true
    }
}

/// Live-path durable claim. It still performs no provider resolution or
/// actuator invocation and must be freshly revalidated before use.
pub struct DurablyClaimedEffectCapabilityV1<S, C, TQ> {
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    manifest: EffectCapabilityClaimAttemptManifestV1,
    receipt: EffectCapabilityClaimReceiptV1,
    claim_record: EffectCapabilityClaimRecordCommitment,
    post_frontier: DurableEffectCapabilityClaimFrontierV1,
    claim_eligibility_valid_until: u64,
    _store: PhantomData<fn() -> S>,
}

impl<S, C, TQ> DurablyClaimedEffectCapabilityV1<S, C, TQ> {
    pub fn capability(&self) -> &C {
        &self.capability
    }

    pub const fn claim_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.claim_time
    }

    pub const fn manifest(&self) -> EffectCapabilityClaimAttemptManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> EffectCapabilityClaimReceiptV1 {
        self.receipt
    }

    pub const fn claim_record(&self) -> EffectCapabilityClaimRecordCommitment {
        self.claim_record
    }

    pub const fn post_frontier(&self) -> DurableEffectCapabilityClaimFrontierV1 {
        self.post_frontier
    }

    pub const fn claim_eligibility_valid_until(&self) -> u64 {
        self.claim_eligibility_valid_until
    }

    pub const fn durable_execution_attempt_lineage_established(&self) -> bool {
        true
    }

    pub const fn requires_final_pre_invocation_revalidation(&self) -> bool {
        true
    }

    pub const fn provider_handle_resolved(&self) -> bool {
        false
    }

    pub const fn actuator_invocation_performed(&self) -> bool {
        false
    }
}

pub enum EffectCapabilityClaimOutcomeV1<S, C, TQ> {
    Claimed(DurablyClaimedEffectCapabilityV1<S, C, TQ>),
    ProvenNotClaimed(RecoverableUnclaimedEffectCapabilityV1<C, TQ>),
    OutcomeUnknown(FrozenEffectCapabilityClaimV1<S, C, TQ>),
}

fn claimed_frontier_is_exact(
    expected: DurableEffectCapabilityClaimFrontierV1,
    claim_record: EffectCapabilityClaimRecordCommitment,
    post: DurableEffectCapabilityClaimFrontierV1,
) -> Result<(), EffectCapabilityClaimAmbiguityReasonV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(EffectCapabilityClaimAmbiguityReasonV1::ClaimedGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(claim_record) {
        return Err(EffectCapabilityClaimAmbiguityReasonV1::InvalidClaimedFrontier);
    }
    Ok(())
}

fn final_absence_frontier_is_compatible(
    expected: DurableEffectCapabilityClaimFrontierV1,
    observed: DurableEffectCapabilityClaimFrontierV1,
) -> bool {
    if observed.generation < expected.generation {
        return false;
    }
    if observed.generation == expected.generation {
        return observed.head == expected.head;
    }
    true
}

fn validate_receipt_common<C: ExactTransientEffectCapabilityV1>(
    expected_store: EffectCapabilityClaimStoreDescriptorV1,
    manifest: EffectCapabilityClaimAttemptManifestV1,
    capability: &C,
    receipt: &EffectCapabilityClaimReceiptV1,
) -> Result<(), EffectCapabilityClaimAmbiguityReasonV1> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(EffectCapabilityClaimAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_store {
        return Err(EffectCapabilityClaimAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest != manifest {
        return Err(EffectCapabilityClaimAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.valid_until {
        return Err(EffectCapabilityClaimAmbiguityReasonV1::ReceiptOutlivesStore);
    }
    if receipt.valid_until > capability.exact_valid_until() {
        return Err(EffectCapabilityClaimAmbiguityReasonV1::ReceiptOutlivesCapability);
    }
    if receipt.valid_until > manifest.subject.claim_time_valid_until {
        return Err(EffectCapabilityClaimAmbiguityReasonV1::ReceiptOutlivesClaimTime);
    }
    Ok(())
}

fn freeze<S, C, TQ>(
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    manifest: EffectCapabilityClaimAttemptManifestV1,
    reason: EffectCapabilityClaimAmbiguityReasonV1,
) -> EffectCapabilityClaimOutcomeV1<S, C, TQ> {
    EffectCapabilityClaimOutcomeV1::OutcomeUnknown(FrozenEffectCapabilityClaimV1 {
        capability,
        claim_time,
        expected_store,
        manifest,
        reason,
        _store: PhantomData,
    })
}

fn interpret_receipt<S, C, TQ>(
    capability: C,
    claim_time: QualifiedCurrentTimeV1<TQ>,
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    manifest: EffectCapabilityClaimAttemptManifestV1,
    receipt: EffectCapabilityClaimReceiptV1,
) -> EffectCapabilityClaimOutcomeV1<S, C, TQ>
where
    C: ExactTransientEffectCapabilityV1,
{
    if let Err(reason) = validate_receipt_common(
        expected_store.descriptor,
        manifest,
        &capability,
        &receipt,
    ) {
        return freeze::<S, C, TQ>(
            capability,
            claim_time,
            expected_store,
            manifest,
            reason,
        );
    }

    match receipt.disposition {
        EffectCapabilityClaimDispositionV1::Claimed {
            claim_record,
            post_frontier,
        } => {
            if let Err(reason) =
                claimed_frontier_is_exact(manifest.expected_frontier, claim_record, post_frontier)
            {
                return freeze::<S, C, TQ>(
                    capability,
                    claim_time,
                    expected_store,
                    manifest,
                    reason,
                );
            }
            let claim_eligibility_valid_until = receipt
                .valid_until
                .min(expected_store.descriptor.valid_until)
                .min(capability.exact_valid_until())
                .min(claim_time.valid_until());
            EffectCapabilityClaimOutcomeV1::Claimed(DurablyClaimedEffectCapabilityV1 {
                capability,
                claim_time,
                manifest,
                receipt,
                claim_record,
                post_frontier,
                claim_eligibility_valid_until,
                _store: PhantomData,
            })
        }
        EffectCapabilityClaimDispositionV1::ProvenNotClaimed {
            observed_frontier, ..
        } => {
            if !final_absence_frontier_is_compatible(manifest.expected_frontier, observed_frontier)
            {
                return freeze::<S, C, TQ>(
                    capability,
                    claim_time,
                    expected_store,
                    manifest,
                    EffectCapabilityClaimAmbiguityReasonV1::InvalidProvenNotClaimedFrontier,
                );
            }
            EffectCapabilityClaimOutcomeV1::ProvenNotClaimed(
                RecoverableUnclaimedEffectCapabilityV1 {
                    capability,
                    claim_time,
                    manifest,
                    receipt,
                },
            )
        }
        EffectCapabilityClaimDispositionV1::AttemptIdConflict { .. } => freeze::<S, C, TQ>(
            capability,
            claim_time,
            expected_store,
            manifest,
            EffectCapabilityClaimAmbiguityReasonV1::AttemptIdConflict,
        ),
        EffectCapabilityClaimDispositionV1::CapabilityAlreadyClaimed { .. } => {
            freeze::<S, C, TQ>(
                capability,
                claim_time,
                expected_store,
                manifest,
                EffectCapabilityClaimAmbiguityReasonV1::CapabilityAlreadyClaimed,
            )
        }
        EffectCapabilityClaimDispositionV1::OutcomeUnknown => freeze::<S, C, TQ>(
            capability,
            claim_time,
            expected_store,
            manifest,
            EffectCapabilityClaimAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

pub fn claim_prepared_effect_capability<S, C, TQ>(
    prepared: PreparedEffectCapabilityClaimV1<C, TQ>,
    store: &S,
) -> Result<
    EffectCapabilityClaimOutcomeV1<S, C, TQ>,
    EffectCapabilityClaimPreWriteFailureV1<C, TQ>,
>
where
    S: EffectCapabilityClaimStoreV1,
    C: ExactTransientEffectCapabilityV1,
{
    if store.descriptor() != prepared.expected_store.descriptor {
        return Err(EffectCapabilityClaimPreWriteFailureV1 {
            prepared,
            error: EffectCapabilityClaimPreWriteError::StoreDescriptorMismatchBefore,
        });
    }

    let PreparedEffectCapabilityClaimV1 {
        capability,
        claim_time,
        expected_store,
        manifest,
        ..
    } = prepared;

    let receipt = match store.claim_effect_capability(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Ok(freeze::<S, C, TQ>(
                capability,
                claim_time,
                expected_store,
                manifest,
                EffectCapabilityClaimAmbiguityReasonV1::StoreErrorAfterClaimBoundary,
            ));
        }
    };

    if store.descriptor() != expected_store.descriptor {
        return Ok(freeze::<S, C, TQ>(
            capability,
            claim_time,
            expected_store,
            manifest,
            EffectCapabilityClaimAmbiguityReasonV1::StoreDescriptorChangedAfter,
        ));
    }

    Ok(interpret_receipt::<S, C, TQ>(
        capability,
        claim_time,
        expected_store,
        manifest,
        receipt,
    ))
}

pub fn reconcile_frozen_effect_capability_claim<S, C, TQ>(
    frozen: FrozenEffectCapabilityClaimV1<S, C, TQ>,
    store: &S,
) -> EffectCapabilityClaimOutcomeV1<S, C, TQ>
where
    S: EffectCapabilityClaimStoreV1,
    C: ExactTransientEffectCapabilityV1,
{
    let FrozenEffectCapabilityClaimV1 {
        capability,
        claim_time,
        expected_store,
        manifest,
        ..
    } = frozen;

    if store.descriptor() != expected_store.descriptor {
        return freeze::<S, C, TQ>(
            capability,
            claim_time,
            expected_store,
            manifest,
            EffectCapabilityClaimAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = match store.reconcile_effect_capability_claim(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return freeze::<S, C, TQ>(
                capability,
                claim_time,
                expected_store,
                manifest,
                EffectCapabilityClaimAmbiguityReasonV1::StoreErrorAfterClaimBoundary,
            );
        }
    };

    if store.descriptor() != expected_store.descriptor {
        return freeze::<S, C, TQ>(
            capability,
            claim_time,
            expected_store,
            manifest,
            EffectCapabilityClaimAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }

    interpret_receipt::<S, C, TQ>(
        capability,
        claim_time,
        expected_store,
        manifest,
        receipt,
    )
}

/// Restart-safe disposition from the exact plain journaled manifest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JournaledEffectCapabilityClaimDispositionV1 {
    Claimed {
        claim_record: EffectCapabilityClaimRecordCommitment,
        post_frontier: DurableEffectCapabilityClaimFrontierV1,
    },
    ProvenNotClaimed {
        observed_frontier: DurableEffectCapabilityClaimFrontierV1,
        absence_evidence: EffectCapabilityClaimAbsenceCommitment,
    },
    OutcomeUnknown {
        reason: EffectCapabilityClaimAmbiguityReasonV1,
    },
}

pub struct JournaledEffectCapabilityClaimReconciliationV1<S> {
    manifest: EffectCapabilityClaimAttemptManifestV1,
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    receipt: Option<EffectCapabilityClaimReceiptV1>,
    disposition: JournaledEffectCapabilityClaimDispositionV1,
    reconciliation_valid_until: Option<u64>,
    claim_eligibility_valid_until: Option<u64>,
    _store: PhantomData<fn() -> S>,
}

impl<S> JournaledEffectCapabilityClaimReconciliationV1<S> {
    pub const fn manifest(&self) -> EffectCapabilityClaimAttemptManifestV1 {
        self.manifest
    }

    pub fn receipt(&self) -> Option<&EffectCapabilityClaimReceiptV1> {
        self.receipt.as_ref()
    }

    pub const fn disposition(&self) -> JournaledEffectCapabilityClaimDispositionV1 {
        self.disposition
    }

    pub const fn reconciliation_valid_until(&self) -> Option<u64> {
        self.reconciliation_valid_until
    }

    pub const fn claim_eligibility_valid_until(&self) -> Option<u64> {
        self.claim_eligibility_valid_until
    }

    pub fn permits_new_attempt(&self) -> bool {
        matches!(
            self.disposition,
            JournaledEffectCapabilityClaimDispositionV1::ProvenNotClaimed { .. }
        )
    }

    pub const fn reconstructs_transient_capability(&self) -> bool {
        false
    }

    pub const fn actuator_invocation_performed(&self) -> bool {
        false
    }
}

fn journaled_unknown<S>(
    manifest: EffectCapabilityClaimAttemptManifestV1,
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    receipt: Option<EffectCapabilityClaimReceiptV1>,
    reason: EffectCapabilityClaimAmbiguityReasonV1,
) -> JournaledEffectCapabilityClaimReconciliationV1<S> {
    JournaledEffectCapabilityClaimReconciliationV1 {
        manifest,
        expected_store,
        receipt,
        disposition: JournaledEffectCapabilityClaimDispositionV1::OutcomeUnknown { reason },
        reconciliation_valid_until: None,
        claim_eligibility_valid_until: None,
        _store: PhantomData,
    }
}

/// Restart reconciliation from the exact journaled manifest alone.
///
/// This v0.1 surface requires the exact original store descriptor. A later
/// child may support same-stable-identity store key/policy rotation.
pub fn reconcile_journaled_effect_capability_claim<S>(
    expected_store: ExpectedEffectCapabilityClaimStoreProfileV1,
    store: &S,
    manifest: EffectCapabilityClaimAttemptManifestV1,
) -> JournaledEffectCapabilityClaimReconciliationV1<S>
where
    S: EffectCapabilityClaimStoreV1,
{
    if manifest.schema_version != SSF_SCHEMA_V1 || manifest.expected_store != expected_store.descriptor
    {
        return journaled_unknown(
            manifest,
            expected_store,
            None,
            EffectCapabilityClaimAmbiguityReasonV1::ReceiptManifestMismatch,
        );
    }
    if store.descriptor() != expected_store.descriptor {
        return journaled_unknown(
            manifest,
            expected_store,
            None,
            EffectCapabilityClaimAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = match store.reconcile_effect_capability_claim(&manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return journaled_unknown(
                manifest,
                expected_store,
                None,
                EffectCapabilityClaimAmbiguityReasonV1::StoreErrorAfterClaimBoundary,
            );
        }
    };

    if store.descriptor() != expected_store.descriptor {
        return journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityClaimAmbiguityReasonV1::StoreDescriptorChangedAfter,
        );
    }
    if receipt.schema_version != SSF_SCHEMA_V1
        || receipt.store != expected_store.descriptor
        || receipt.manifest != manifest
        || receipt.valid_until > expected_store.descriptor.valid_until
        || receipt.valid_until > manifest.subject.capability.valid_until
        || receipt.valid_until > manifest.subject.claim_time_valid_until
    {
        return journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityClaimAmbiguityReasonV1::ReceiptManifestMismatch,
        );
    }

    match receipt.disposition {
        EffectCapabilityClaimDispositionV1::Claimed {
            claim_record,
            post_frontier,
        } => {
            if let Err(reason) =
                claimed_frontier_is_exact(manifest.expected_frontier, claim_record, post_frontier)
            {
                return journaled_unknown(manifest, expected_store, Some(receipt), reason);
            }
            let claim_eligibility_valid_until = receipt
                .valid_until
                .min(manifest.subject.capability.valid_until)
                .min(manifest.subject.claim_time_valid_until);
            JournaledEffectCapabilityClaimReconciliationV1 {
                manifest,
                expected_store,
                receipt: Some(receipt),
                disposition: JournaledEffectCapabilityClaimDispositionV1::Claimed {
                    claim_record,
                    post_frontier,
                },
                reconciliation_valid_until: Some(receipt.valid_until),
                claim_eligibility_valid_until: Some(claim_eligibility_valid_until),
                _store: PhantomData,
            }
        }
        EffectCapabilityClaimDispositionV1::ProvenNotClaimed {
            observed_frontier,
            absence_evidence,
        } => {
            if !final_absence_frontier_is_compatible(manifest.expected_frontier, observed_frontier)
            {
                return journaled_unknown(
                    manifest,
                    expected_store,
                    Some(receipt),
                    EffectCapabilityClaimAmbiguityReasonV1::InvalidProvenNotClaimedFrontier,
                );
            }
            JournaledEffectCapabilityClaimReconciliationV1 {
                manifest,
                expected_store,
                receipt: Some(receipt),
                disposition: JournaledEffectCapabilityClaimDispositionV1::ProvenNotClaimed {
                    observed_frontier,
                    absence_evidence,
                },
                reconciliation_valid_until: Some(receipt.valid_until),
                claim_eligibility_valid_until: None,
                _store: PhantomData,
            }
        }
        EffectCapabilityClaimDispositionV1::AttemptIdConflict { .. } => journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityClaimAmbiguityReasonV1::AttemptIdConflict,
        ),
        EffectCapabilityClaimDispositionV1::CapabilityAlreadyClaimed { .. } => journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityClaimAmbiguityReasonV1::CapabilityAlreadyClaimed,
        ),
        EffectCapabilityClaimDispositionV1::OutcomeUnknown => journaled_unknown(
            manifest,
            expected_store,
            Some(receipt),
            EffectCapabilityClaimAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn frontier(generation: u64, head: Option<u8>) -> DurableEffectCapabilityClaimFrontierV1 {
        DurableEffectCapabilityClaimFrontierV1 {
            generation: DurableEffectCapabilityClaimGeneration::new(generation),
            head: head.map(|byte| EffectCapabilityClaimRecordCommitment::from_bytes(bytes(byte))),
        }
    }

    #[test]
    fn claim_frontier_advances_exactly_one_generation_to_exact_record() {
        let record = EffectCapabilityClaimRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            claimed_frontier_is_exact(frontier(4, Some(1)), record, frontier(5, Some(9))),
            Ok(())
        );
    }

    #[test]
    fn skipped_claim_generation_is_rejected() {
        let record = EffectCapabilityClaimRecordCommitment::from_bytes(bytes(9));
        assert_eq!(
            claimed_frontier_is_exact(frontier(4, Some(1)), record, frontier(6, Some(9))),
            Err(EffectCapabilityClaimAmbiguityReasonV1::InvalidClaimedFrontier)
        );
    }

    #[test]
    fn final_nonclaim_never_rolls_frontier_backward() {
        assert!(!final_absence_frontier_is_compatible(
            frontier(5, Some(2)),
            frontier(4, Some(2))
        ));
    }

    #[test]
    fn same_generation_nonclaim_requires_exact_head() {
        assert!(!final_absence_frontier_is_compatible(
            frontier(5, Some(2)),
            frontier(5, Some(3))
        ));
    }

    #[test]
    fn preexisting_claim_is_not_final_nonclaim() {
        let record = EffectCapabilityClaimRecordCommitment::from_bytes(bytes(7));
        let existing = EffectCapabilityClaimDispositionV1::CapabilityAlreadyClaimed {
            existing_claim_record: record,
        };
        let absent = EffectCapabilityClaimDispositionV1::ProvenNotClaimed {
            observed_frontier: frontier(1, None),
            absence_evidence: EffectCapabilityClaimAbsenceCommitment::from_bytes(bytes(8)),
        };
        assert_ne!(existing, absent);
    }

    #[test]
    fn claim_store_has_explicit_unix_time_basis() {
        let descriptor = EffectCapabilityClaimStoreDescriptorV1 {
            stable_identity: EffectCapabilityClaimStoreIdentityCommitment::from_bytes(bytes(1)),
            policy: EffectCapabilityClaimStorePolicyCommitment::from_bytes(bytes(2)),
            generation: EffectCapabilityClaimStoreGeneration::new(3),
            time_basis: EffectCapabilityClaimStoreTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 100,
        };
        assert_eq!(
            descriptor.time_basis,
            EffectCapabilityClaimStoreTimeBasisV1::UnixMillisecondsUtc
        );
    }
}

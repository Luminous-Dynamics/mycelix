// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Crash-safe durable journal of one exact actuator invocation attempt.
//!
//! v0.1 permits at most one journaled invocation attempt per durable SSF claim
//! record, regardless of actuator replay mode. This makes restart recovery by
//! claim record unambiguous before the system has durable outcome/replay
//! authorization semantics.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    ActuatorInvocationAttemptId, ActuatorInvocationAttemptManifestV1,
    PreparedActuatorEffectAttemptV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_durable_effect_capability_claim::EffectCapabilityClaimRecordCommitment;

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

digest_type!(InvocationAttemptJournalRecordCommitment);
digest_type!(InvocationAttemptJournalReceiptCommitment);
digest_type!(InvocationAttemptJournalAbsenceCommitment);
digest_type!(InvocationAttemptJournalManifestCommitment);
digest_type!(InvocationAttemptJournalStoreIdentityCommitment);
digest_type!(InvocationAttemptJournalStorePolicyCommitment);

generation_type!(InvocationAttemptJournalStoreGeneration);
generation_type!(DurableInvocationAttemptJournalGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvocationAttemptJournalTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InvocationAttemptJournalStoreDescriptorV1 {
    pub stable_identity: InvocationAttemptJournalStoreIdentityCommitment,
    pub policy: InvocationAttemptJournalStorePolicyCommitment,
    pub generation: InvocationAttemptJournalStoreGeneration,
    pub time_basis: InvocationAttemptJournalTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedInvocationAttemptJournalStoreProfileV1 {
    descriptor: InvocationAttemptJournalStoreDescriptorV1,
}

impl ExpectedInvocationAttemptJournalStoreProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: InvocationAttemptJournalStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> InvocationAttemptJournalStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DurableInvocationAttemptJournalFrontierV1 {
    pub generation: DurableInvocationAttemptJournalGeneration,
    pub head: Option<InvocationAttemptJournalRecordCommitment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedDurableInvocationAttemptJournalFrontierV1 {
    frontier: DurableInvocationAttemptJournalFrontierV1,
}

impl ExpectedDurableInvocationAttemptJournalFrontierV1 {
    pub const fn from_trusted_state(frontier: DurableInvocationAttemptJournalFrontierV1) -> Self {
        Self { frontier }
    }

    pub const fn frontier(&self) -> DurableInvocationAttemptJournalFrontierV1 {
        self.frontier
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvocationAttemptJournalManifestV1 {
    pub schema_version: u16,
    pub attempt: ActuatorInvocationAttemptManifestV1,
    pub expected_store: InvocationAttemptJournalStoreDescriptorV1,
    pub expected_frontier: DurableInvocationAttemptJournalFrontierV1,
}

impl InvocationAttemptJournalManifestV1 {
    pub const fn claim_record(&self) -> EffectCapabilityClaimRecordCommitment {
        self.attempt.subject.claim_record
    }

    pub const fn attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.attempt.attempt_id
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvocationAttemptJournalDispositionV1 {
    Journaled {
        record: InvocationAttemptJournalRecordCommitment,
        post_frontier: DurableInvocationAttemptJournalFrontierV1,
    },
    ProvenNotJournaled {
        observed_frontier: DurableInvocationAttemptJournalFrontierV1,
        absence_evidence: InvocationAttemptJournalAbsenceCommitment,
    },
    AttemptIdConflict {
        existing_manifest: InvocationAttemptJournalManifestCommitment,
    },
    ClaimAlreadyHasAttempt {
        existing_record: InvocationAttemptJournalRecordCommitment,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvocationAttemptJournalReceiptV1 {
    pub schema_version: u16,
    pub store: InvocationAttemptJournalStoreDescriptorV1,
    pub manifest: InvocationAttemptJournalManifestV1,
    pub disposition: InvocationAttemptJournalDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: InvocationAttemptJournalReceiptCommitment,
}

/// Durable invocation-attempt journal.
///
/// Implementations MUST enforce:
///
/// - first-seen attempt ID permanently binds the exact manifest;
/// - one durable claim record has at most one journaled invocation attempt in
///   v0.1, even if the actuator is idempotent;
/// - `ProvenNotJournaled` is final for that attempt ID;
/// - all reconcile methods are read-only with respect to the external effect.
pub trait InvocationAttemptJournalStoreV1 {
    type Error;

    fn descriptor(&self) -> InvocationAttemptJournalStoreDescriptorV1;

    fn journal_attempt(
        &self,
        manifest: &InvocationAttemptJournalManifestV1,
    ) -> Result<InvocationAttemptJournalReceiptV1, Self::Error>;

    fn reconcile_attempt(
        &self,
        manifest: &InvocationAttemptJournalManifestV1,
    ) -> Result<InvocationAttemptJournalReceiptV1, Self::Error>;

    /// Safe in v0.1 because one claim record may have at most one journaled
    /// invocation attempt. Future multi-attempt replay requires a different
    /// history interface.
    fn reconcile_claim_record(
        &self,
        claim_record: EffectCapabilityClaimRecordCommitment,
    ) -> Result<InvocationAttemptJournalReceiptV1, Self::Error>;
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactPreparedActuatorEffectAttemptV1: sealed::Sealed {
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1;
}

impl<Q, TQ, ITQ> sealed::Sealed for PreparedActuatorEffectAttemptV1<Q, TQ, ITQ> {}

impl<Q, TQ, ITQ> ExactPreparedActuatorEffectAttemptV1
    for PreparedActuatorEffectAttemptV1<Q, TQ, ITQ>
{
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.manifest()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvocationAttemptJournalPreparationError {
    UnsupportedStoreTimeBasis,
    StoreAlreadyExpiredForAttempt,
}

pub struct PreparedInvocationAttemptJournalV1<P> {
    prepared_attempt: P,
    expected_store: ExpectedInvocationAttemptJournalStoreProfileV1,
    manifest: InvocationAttemptJournalManifestV1,
}

impl<P> PreparedInvocationAttemptJournalV1<P> {
    pub fn prepared_attempt(&self) -> &P {
        &self.prepared_attempt
    }

    pub const fn expected_store(&self) -> ExpectedInvocationAttemptJournalStoreProfileV1 {
        self.expected_store
    }

    pub const fn manifest(&self) -> InvocationAttemptJournalManifestV1 {
        self.manifest
    }
}

pub struct InvocationAttemptJournalPreparationFailureV1<P> {
    prepared_attempt: P,
    error: InvocationAttemptJournalPreparationError,
}

impl<P> InvocationAttemptJournalPreparationFailureV1<P> {
    pub const fn error(&self) -> InvocationAttemptJournalPreparationError {
        self.error
    }

    pub fn into_prepared_attempt(self) -> P {
        self.prepared_attempt
    }
}

pub fn prepare_invocation_attempt_journal<P>(
    prepared_attempt: P,
    expected_store: ExpectedInvocationAttemptJournalStoreProfileV1,
    expected_frontier: ExpectedDurableInvocationAttemptJournalFrontierV1,
) -> Result<PreparedInvocationAttemptJournalV1<P>, InvocationAttemptJournalPreparationFailureV1<P>>
where
    P: ExactPreparedActuatorEffectAttemptV1,
{
    let attempt = prepared_attempt.exact_attempt_manifest();
    let error = if expected_store.descriptor.time_basis
        != InvocationAttemptJournalTimeBasisV1::UnixMillisecondsUtc
    {
        Some(InvocationAttemptJournalPreparationError::UnsupportedStoreTimeBasis)
    } else if expected_store.descriptor.valid_until < attempt.latest_possible_unix_ms {
        Some(InvocationAttemptJournalPreparationError::StoreAlreadyExpiredForAttempt)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(InvocationAttemptJournalPreparationFailureV1 {
            prepared_attempt,
            error,
        });
    }

    let manifest = InvocationAttemptJournalManifestV1 {
        schema_version: SSF_SCHEMA_V1,
        attempt,
        expected_store: expected_store.descriptor,
        expected_frontier: expected_frontier.frontier,
    };

    Ok(PreparedInvocationAttemptJournalV1 {
        prepared_attempt,
        expected_store,
        manifest,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvocationAttemptJournalAmbiguityReasonV1 {
    StoreErrorAfterJournalBoundary,
    StoreErrorDuringReconciliation,
    StoreDescriptorChangedAfter,
    StoreDescriptorMismatchBeforeReconciliation,
    UnsupportedReceiptSchema,
    UnsupportedManifestSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    JournalGenerationOverflow,
    InvalidJournaledFrontier,
    InvalidProvenNotJournaledFrontier,
    AttemptIdConflict,
    ClaimAlreadyHasAttempt,
    StoreReportedOutcomeUnknown,
}

pub struct DurablyJournaledActuatorEffectAttemptV1<S, P> {
    prepared_attempt: P,
    manifest: InvocationAttemptJournalManifestV1,
    receipt: InvocationAttemptJournalReceiptV1,
    record: InvocationAttemptJournalRecordCommitment,
    post_frontier: DurableInvocationAttemptJournalFrontierV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, P> DurablyJournaledActuatorEffectAttemptV1<S, P> {
    pub fn prepared_attempt(&self) -> &P {
        &self.prepared_attempt
    }

    pub const fn manifest(&self) -> InvocationAttemptJournalManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> InvocationAttemptJournalReceiptV1 {
        self.receipt
    }

    pub const fn record(&self) -> InvocationAttemptJournalRecordCommitment {
        self.record
    }

    pub const fn post_frontier(&self) -> DurableInvocationAttemptJournalFrontierV1 {
        self.post_frontier
    }

    pub const fn eligible_for_actuator_invocation_protocol(&self) -> bool {
        true
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }
}

pub struct FrozenInvocationAttemptJournalV1<S, P> {
    prepared_attempt: P,
    manifest: InvocationAttemptJournalManifestV1,
    expected_store: ExpectedInvocationAttemptJournalStoreProfileV1,
    reason: InvocationAttemptJournalAmbiguityReasonV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, P> FrozenInvocationAttemptJournalV1<S, P> {
    pub fn prepared_attempt(&self) -> &P {
        &self.prepared_attempt
    }

    pub const fn manifest(&self) -> InvocationAttemptJournalManifestV1 {
        self.manifest
    }

    pub const fn expected_store(&self) -> ExpectedInvocationAttemptJournalStoreProfileV1 {
        self.expected_store
    }

    pub const fn reason(&self) -> InvocationAttemptJournalAmbiguityReasonV1 {
        self.reason
    }

    pub const fn may_create_fresh_attempt(&self) -> bool {
        false
    }

    pub const fn external_effect_may_have_occurred(&self) -> bool {
        false
    }
}

pub struct RecoverableUnjournaledActuatorEffectAttemptV1<P> {
    prepared_attempt: P,
    receipt: InvocationAttemptJournalReceiptV1,
}

impl<P> RecoverableUnjournaledActuatorEffectAttemptV1<P> {
    pub fn into_prepared_attempt(self) -> P {
        self.prepared_attempt
    }

    pub const fn receipt(&self) -> InvocationAttemptJournalReceiptV1 {
        self.receipt
    }
}

pub enum InvocationAttemptJournalOutcomeV1<S, P> {
    Journaled(DurablyJournaledActuatorEffectAttemptV1<S, P>),
    ProvenNotJournaled(RecoverableUnjournaledActuatorEffectAttemptV1<P>),
    OutcomeUnknown(FrozenInvocationAttemptJournalV1<S, P>),
}

fn validate_journaled_frontier(
    expected: DurableInvocationAttemptJournalFrontierV1,
    record: InvocationAttemptJournalRecordCommitment,
    post: DurableInvocationAttemptJournalFrontierV1,
) -> Result<(), InvocationAttemptJournalAmbiguityReasonV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(InvocationAttemptJournalAmbiguityReasonV1::JournalGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(record) {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::InvalidJournaledFrontier);
    }
    Ok(())
}

fn absence_frontier_is_compatible(
    expected: DurableInvocationAttemptJournalFrontierV1,
    observed: DurableInvocationAttemptJournalFrontierV1,
) -> bool {
    if observed.generation < expected.generation {
        return false;
    }
    if observed.generation == expected.generation {
        return observed.head == expected.head;
    }
    true
}

fn freeze_from_prepared<S, P>(
    prepared: PreparedInvocationAttemptJournalV1<P>,
    reason: InvocationAttemptJournalAmbiguityReasonV1,
) -> InvocationAttemptJournalOutcomeV1<S, P> {
    InvocationAttemptJournalOutcomeV1::OutcomeUnknown(FrozenInvocationAttemptJournalV1 {
        prepared_attempt: prepared.prepared_attempt,
        manifest: prepared.manifest,
        expected_store: prepared.expected_store,
        reason,
        _store: PhantomData,
    })
}

fn interpret_receipt<S, P>(
    prepared: PreparedInvocationAttemptJournalV1<P>,
    receipt: InvocationAttemptJournalReceiptV1,
) -> InvocationAttemptJournalOutcomeV1<S, P> {
    let manifest = prepared.manifest;

    if receipt.schema_version != SSF_SCHEMA_V1 {
        return freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::UnsupportedReceiptSchema,
        );
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1 {
        return freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }
    if receipt.store != prepared.expected_store.descriptor {
        return freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::ReceiptStoreMismatch,
        );
    }
    if receipt.manifest != manifest {
        return freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::ReceiptManifestMismatch,
        );
    }
    if receipt.valid_until > prepared.expected_store.descriptor.valid_until {
        return freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::ReceiptOutlivesStore,
        );
    }

    match receipt.disposition {
        InvocationAttemptJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => {
            if let Err(reason) =
                validate_journaled_frontier(manifest.expected_frontier, record, post_frontier)
            {
                return freeze_from_prepared(prepared, reason);
            }
            InvocationAttemptJournalOutcomeV1::Journaled(
                DurablyJournaledActuatorEffectAttemptV1 {
                    prepared_attempt: prepared.prepared_attempt,
                    manifest,
                    receipt,
                    record,
                    post_frontier,
                    _store: PhantomData,
                },
            )
        }
        InvocationAttemptJournalDispositionV1::ProvenNotJournaled {
            observed_frontier,
            ..
        } => {
            if !absence_frontier_is_compatible(manifest.expected_frontier, observed_frontier) {
                return freeze_from_prepared(
                    prepared,
                    InvocationAttemptJournalAmbiguityReasonV1::InvalidProvenNotJournaledFrontier,
                );
            }
            InvocationAttemptJournalOutcomeV1::ProvenNotJournaled(
                RecoverableUnjournaledActuatorEffectAttemptV1 {
                    prepared_attempt: prepared.prepared_attempt,
                    receipt,
                },
            )
        }
        InvocationAttemptJournalDispositionV1::AttemptIdConflict { .. } => freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::AttemptIdConflict,
        ),
        InvocationAttemptJournalDispositionV1::ClaimAlreadyHasAttempt { .. } => {
            freeze_from_prepared(
                prepared,
                InvocationAttemptJournalAmbiguityReasonV1::ClaimAlreadyHasAttempt,
            )
        }
        InvocationAttemptJournalDispositionV1::OutcomeUnknown => freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

pub fn journal_invocation_attempt<S, P>(
    prepared: PreparedInvocationAttemptJournalV1<P>,
    store: &S,
) -> Result<InvocationAttemptJournalOutcomeV1<S, P>, PreparedInvocationAttemptJournalV1<P>>
where
    S: InvocationAttemptJournalStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor {
        return Err(prepared);
    }

    let receipt = match store.journal_attempt(&prepared.manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Ok(freeze_from_prepared(
                prepared,
                InvocationAttemptJournalAmbiguityReasonV1::StoreErrorAfterJournalBoundary,
            ));
        }
    };

    if store.descriptor() != prepared.expected_store.descriptor {
        return Ok(freeze_from_prepared(
            prepared,
            InvocationAttemptJournalAmbiguityReasonV1::StoreDescriptorChangedAfter,
        ));
    }

    Ok(interpret_receipt::<S, P>(prepared, receipt))
}

/// Read-only restart evidence recovered solely from the durable claim record.
///
/// This does not reconstruct the fresh pre-invocation qualification. A later
/// rebind/requalification step is still required before external invocation.
pub struct RecoveredInvocationAttemptJournalV1<S> {
    receipt: InvocationAttemptJournalReceiptV1,
    _store: PhantomData<fn() -> S>,
}

impl<S> RecoveredInvocationAttemptJournalV1<S> {
    pub const fn receipt(&self) -> InvocationAttemptJournalReceiptV1 {
        self.receipt
    }

    pub const fn external_effect_attempted_by_recovery(&self) -> bool {
        false
    }
}

pub fn recover_journal_by_claim_record<S>(
    store: &S,
    expected_store: ExpectedInvocationAttemptJournalStoreProfileV1,
    claim_record: EffectCapabilityClaimRecordCommitment,
) -> Result<RecoveredInvocationAttemptJournalV1<S>, InvocationAttemptJournalAmbiguityReasonV1>
where
    S: InvocationAttemptJournalStoreV1,
{
    if store.descriptor() != expected_store.descriptor {
        return Err(
            InvocationAttemptJournalAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = store
        .reconcile_claim_record(claim_record)
        .map_err(|_| InvocationAttemptJournalAmbiguityReasonV1::StoreErrorDuringReconciliation)?;

    if store.descriptor() != expected_store.descriptor {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::StoreDescriptorChangedAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1
        || receipt.manifest.attempt.schema_version != SSF_SCHEMA_V1
    {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::UnsupportedManifestSchema);
    }
    if receipt.store != expected_store.descriptor {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest.expected_store != expected_store.descriptor {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.manifest.claim_record() != claim_record {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.descriptor.valid_until {
        return Err(InvocationAttemptJournalAmbiguityReasonV1::ReceiptOutlivesStore);
    }

    match receipt.disposition {
        InvocationAttemptJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => {
            validate_journaled_frontier(receipt.manifest.expected_frontier, record, post_frontier)?;
            Ok(RecoveredInvocationAttemptJournalV1 {
                receipt,
                _store: PhantomData,
            })
        }
        InvocationAttemptJournalDispositionV1::ProvenNotJournaled { .. }
        | InvocationAttemptJournalDispositionV1::AttemptIdConflict { .. }
        | InvocationAttemptJournalDispositionV1::ClaimAlreadyHasAttempt { .. }
        | InvocationAttemptJournalDispositionV1::OutcomeUnknown => Err(
            InvocationAttemptJournalAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frontier(
        generation: u64,
        head: Option<InvocationAttemptJournalRecordCommitment>,
    ) -> DurableInvocationAttemptJournalFrontierV1 {
        DurableInvocationAttemptJournalFrontierV1 {
            generation: DurableInvocationAttemptJournalGeneration::new(generation),
            head,
        }
    }

    #[test]
    fn exact_journal_frontier_advances_once() {
        let record = InvocationAttemptJournalRecordCommitment::from_bytes([9; 32]);
        assert_eq!(
            validate_journaled_frontier(frontier(4, None), record, frontier(5, Some(record))),
            Ok(())
        );
    }

    #[test]
    fn skipped_journal_generation_is_rejected() {
        let record = InvocationAttemptJournalRecordCommitment::from_bytes([9; 32]);
        assert_eq!(
            validate_journaled_frontier(frontier(4, None), record, frontier(6, Some(record))),
            Err(InvocationAttemptJournalAmbiguityReasonV1::InvalidJournaledFrontier)
        );
    }

    #[test]
    fn absence_cannot_roll_frontier_backward() {
        assert!(!absence_frontier_is_compatible(
            frontier(5, None),
            frontier(4, None)
        ));
    }
}

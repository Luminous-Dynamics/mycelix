// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Crash-safe journal for one explicitly authorized SSF replay attempt.
//!
//! The original invocation journal remains single-attempt. This crate adds a
//! separate replay lane that can create one second attempt only by consuming an
//! `AuthorizedReplayV1` plus a newly prepared, freshly qualified actuator
//! attempt with the exact same stable effect identity.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    ActuatorInvocationAttemptId, ActuatorInvocationAttemptManifestV1,
    PreparedActuatorEffectAttemptV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_durable_actuator_invocation_attempt::InvocationAttemptJournalRecordCommitment;
use mycelix_ssf_replay_authorization::{
    AuthorizedReplayV1, ReplayAuthorizationCommitment, ReplayAuthorizationSubjectV1,
    ReplayPolicyReceiptV1,
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

digest_type!(ReplayInvocationJournalRecordCommitment);
digest_type!(ReplayInvocationJournalReceiptCommitment);
digest_type!(ReplayInvocationJournalAbsenceCommitment);
digest_type!(ReplayInvocationJournalManifestCommitment);
digest_type!(ReplayInvocationJournalStoreIdentityCommitment);
digest_type!(ReplayInvocationJournalStorePolicyCommitment);

generation_type!(ReplayInvocationJournalStoreGeneration);
generation_type!(DurableReplayInvocationJournalGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplayInvocationJournalTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ReplayInvocationJournalStoreDescriptorV1 {
    pub stable_identity: ReplayInvocationJournalStoreIdentityCommitment,
    pub policy: ReplayInvocationJournalStorePolicyCommitment,
    pub generation: ReplayInvocationJournalStoreGeneration,
    pub time_basis: ReplayInvocationJournalTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedReplayInvocationJournalStoreProfileV1 {
    descriptor: ReplayInvocationJournalStoreDescriptorV1,
}

impl ExpectedReplayInvocationJournalStoreProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: ReplayInvocationJournalStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> ReplayInvocationJournalStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DurableReplayInvocationJournalFrontierV1 {
    pub generation: DurableReplayInvocationJournalGeneration,
    pub head: Option<ReplayInvocationJournalRecordCommitment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedDurableReplayInvocationJournalFrontierV1 {
    frontier: DurableReplayInvocationJournalFrontierV1,
}

impl ExpectedDurableReplayInvocationJournalFrontierV1 {
    pub const fn from_trusted_state(frontier: DurableReplayInvocationJournalFrontierV1) -> Self {
        Self { frontier }
    }

    pub const fn frontier(&self) -> DurableReplayInvocationJournalFrontierV1 {
        self.frontier
    }
}

/// Copyable evidence binding extracted from the non-copyable replay token.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayAuthorizationBindingV1 {
    pub subject: ReplayAuthorizationSubjectV1,
    pub policy_receipt: ReplayPolicyReceiptV1,
    pub authorization: ReplayAuthorizationCommitment,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayInvocationJournalManifestV1 {
    pub schema_version: u16,
    pub replay: ReplayAuthorizationBindingV1,
    pub attempt: ActuatorInvocationAttemptManifestV1,
    pub expected_store: ReplayInvocationJournalStoreDescriptorV1,
    pub expected_frontier: DurableReplayInvocationJournalFrontierV1,
}

impl ReplayInvocationJournalManifestV1 {
    pub const fn attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.attempt.attempt_id
    }

    pub const fn prior_attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.replay.subject.prior_attempt_id
    }

    pub const fn prior_journal_record(&self) -> InvocationAttemptJournalRecordCommitment {
        self.replay.subject.history_head.journal_record()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayInvocationJournalDispositionV1 {
    Journaled {
        record: ReplayInvocationJournalRecordCommitment,
        post_frontier: DurableReplayInvocationJournalFrontierV1,
    },
    ProvenNotJournaled {
        observed_frontier: DurableReplayInvocationJournalFrontierV1,
        absence_evidence: ReplayInvocationJournalAbsenceCommitment,
    },
    AttemptIdConflict {
        existing_manifest: ReplayInvocationJournalManifestCommitment,
    },
    ReplayAuthorizationAlreadyConsumed {
        existing_record: ReplayInvocationJournalRecordCommitment,
    },
    PriorAttemptAlreadyReplayed {
        existing_record: ReplayInvocationJournalRecordCommitment,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayInvocationJournalReceiptV1 {
    pub schema_version: u16,
    pub store: ReplayInvocationJournalStoreDescriptorV1,
    pub manifest: ReplayInvocationJournalManifestV1,
    pub disposition: ReplayInvocationJournalDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: ReplayInvocationJournalReceiptCommitment,
}

/// Durable replay journal contract.
///
/// Implementations MUST enforce:
///
/// - first-seen new attempt ID permanently binds its exact manifest;
/// - one replay authorization commitment may produce at most one journaled
///   replay attempt;
/// - one prior attempt may produce at most one replay record in v0.1;
/// - `ProvenNotJournaled` is final for that new attempt ID;
/// - reconciliation methods are non-effecting.
pub trait ReplayInvocationJournalStoreV1 {
    type Error;

    fn descriptor(&self) -> ReplayInvocationJournalStoreDescriptorV1;

    fn journal_replay(
        &self,
        manifest: &ReplayInvocationJournalManifestV1,
    ) -> Result<ReplayInvocationJournalReceiptV1, Self::Error>;

    fn reconcile_replay(
        &self,
        manifest: &ReplayInvocationJournalManifestV1,
    ) -> Result<ReplayInvocationJournalReceiptV1, Self::Error>;

    /// Safe in v0.1 because one prior attempt may have at most one replay.
    fn reconcile_prior_attempt(
        &self,
        prior_attempt_id: ActuatorInvocationAttemptId,
    ) -> Result<ReplayInvocationJournalReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayInvocationJournalPreparationErrorV1 {
    UnsupportedStoreTimeBasis,
    UnsupportedAttemptSchema,
    ReplayStableIdentityIncoherent,
    StableEffectIdentityMismatch,
    ReusedPriorAttemptId,
    ReplayAuthorizationAlreadyExpired,
    PreparedAttemptAlreadyExpired,
    StoreAlreadyExpired,
}

pub struct PreparedReplayInvocationJournalV1<P, S, TQ, Q, QTQ, ITQ> {
    authorized_replay: AuthorizedReplayV1<P, S, TQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    manifest: ReplayInvocationJournalManifestV1,
}

impl<P, S, TQ, Q, QTQ, ITQ> PreparedReplayInvocationJournalV1<P, S, TQ, Q, QTQ, ITQ> {
    pub const fn authorized_replay(&self) -> &AuthorizedReplayV1<P, S, TQ> {
        &self.authorized_replay
    }

    pub const fn prepared_attempt(&self) -> &PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ> {
        &self.prepared_attempt
    }

    pub const fn expected_store(&self) -> ExpectedReplayInvocationJournalStoreProfileV1 {
        self.expected_store
    }

    pub const fn manifest(&self) -> ReplayInvocationJournalManifestV1 {
        self.manifest
    }
}

pub struct ReplayInvocationJournalPreparationFailureV1<P, S, TQ, Q, QTQ, ITQ> {
    authorized_replay: AuthorizedReplayV1<P, S, TQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    error: ReplayInvocationJournalPreparationErrorV1,
}

impl<P, S, TQ, Q, QTQ, ITQ>
    ReplayInvocationJournalPreparationFailureV1<P, S, TQ, Q, QTQ, ITQ>
{
    pub const fn error(&self) -> ReplayInvocationJournalPreparationErrorV1 {
        self.error
    }

    pub fn into_parts(
        self,
    ) -> (
        AuthorizedReplayV1<P, S, TQ>,
        PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    ) {
        (self.authorized_replay, self.prepared_attempt)
    }
}

pub fn prepare_replay_invocation_journal<P, S, TQ, Q, QTQ, ITQ>(
    authorized_replay: AuthorizedReplayV1<P, S, TQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    expected_frontier: ExpectedDurableReplayInvocationJournalFrontierV1,
) -> Result<
    PreparedReplayInvocationJournalV1<P, S, TQ, Q, QTQ, ITQ>,
    ReplayInvocationJournalPreparationFailureV1<P, S, TQ, Q, QTQ, ITQ>,
> {
    let attempt = prepared_attempt.manifest();
    let replay_subject = authorized_replay.subject();
    let latest = attempt.latest_possible_unix_ms;

    let error = if expected_store.descriptor.time_basis
        != ReplayInvocationJournalTimeBasisV1::UnixMillisecondsUtc
    {
        Some(ReplayInvocationJournalPreparationErrorV1::UnsupportedStoreTimeBasis)
    } else if attempt.schema_version != SSF_SCHEMA_V1 {
        Some(ReplayInvocationJournalPreparationErrorV1::UnsupportedAttemptSchema)
    } else if replay_subject.prior_effect_subject.stable_identity()
        != replay_subject.stable_effect_identity
    {
        Some(ReplayInvocationJournalPreparationErrorV1::ReplayStableIdentityIncoherent)
    } else if attempt.subject.stable_identity() != replay_subject.stable_effect_identity {
        Some(ReplayInvocationJournalPreparationErrorV1::StableEffectIdentityMismatch)
    } else if attempt.attempt_id == replay_subject.prior_attempt_id {
        Some(ReplayInvocationJournalPreparationErrorV1::ReusedPriorAttemptId)
    } else if authorized_replay.valid_until() < latest {
        Some(ReplayInvocationJournalPreparationErrorV1::ReplayAuthorizationAlreadyExpired)
    } else if attempt.attempt_valid_until < latest {
        Some(ReplayInvocationJournalPreparationErrorV1::PreparedAttemptAlreadyExpired)
    } else if expected_store.descriptor.valid_until < latest {
        Some(ReplayInvocationJournalPreparationErrorV1::StoreAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(ReplayInvocationJournalPreparationFailureV1 {
            authorized_replay,
            prepared_attempt,
            error,
        });
    }

    let replay = ReplayAuthorizationBindingV1 {
        subject: replay_subject,
        policy_receipt: authorized_replay.receipt(),
        authorization: authorized_replay.authorization(),
        valid_until: authorized_replay.valid_until(),
    };
    let manifest = ReplayInvocationJournalManifestV1 {
        schema_version: SSF_SCHEMA_V1,
        replay,
        attempt,
        expected_store: expected_store.descriptor,
        expected_frontier: expected_frontier.frontier,
    };

    Ok(PreparedReplayInvocationJournalV1 {
        authorized_replay,
        prepared_attempt,
        expected_store,
        manifest,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplayInvocationJournalAmbiguityReasonV1 {
    StoreErrorAfterJournalBoundary,
    StoreErrorDuringReconciliation,
    StoreDescriptorChangedAfter,
    StoreDescriptorMismatchBeforeReconciliation,
    UnsupportedReceiptSchema,
    UnsupportedManifestSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    ReceiptAlreadyExpired,
    JournalGenerationOverflow,
    InvalidJournaledFrontier,
    InvalidProvenNotJournaledFrontier,
    AttemptIdConflict,
    ReplayAuthorizationAlreadyConsumed,
    PriorAttemptAlreadyReplayed,
    StoreReportedOutcomeUnknown,
}

pub struct DurablyJournaledReplayAttemptV1<R, P, S, TQ, Q, QTQ, ITQ> {
    authorized_replay: AuthorizedReplayV1<P, S, TQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    manifest: ReplayInvocationJournalManifestV1,
    receipt: ReplayInvocationJournalReceiptV1,
    record: ReplayInvocationJournalRecordCommitment,
    post_frontier: DurableReplayInvocationJournalFrontierV1,
    _store: PhantomData<fn() -> R>,
}

impl<R, P, S, TQ, Q, QTQ, ITQ>
    DurablyJournaledReplayAttemptV1<R, P, S, TQ, Q, QTQ, ITQ>
{
    pub const fn authorized_replay(&self) -> &AuthorizedReplayV1<P, S, TQ> {
        &self.authorized_replay
    }

    pub const fn prepared_attempt(&self) -> &PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ> {
        &self.prepared_attempt
    }

    pub const fn manifest(&self) -> ReplayInvocationJournalManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> ReplayInvocationJournalReceiptV1 {
        self.receipt
    }

    pub const fn record(&self) -> ReplayInvocationJournalRecordCommitment {
        self.record
    }

    pub const fn post_frontier(&self) -> DurableReplayInvocationJournalFrontierV1 {
        self.post_frontier
    }

    pub const fn eligible_for_replay_actuator_invocation_protocol(&self) -> bool {
        true
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

pub struct FrozenReplayInvocationJournalV1<R, P, S, TQ, Q, QTQ, ITQ> {
    authorized_replay: AuthorizedReplayV1<P, S, TQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    manifest: ReplayInvocationJournalManifestV1,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    reason: ReplayInvocationJournalAmbiguityReasonV1,
    _store: PhantomData<fn() -> R>,
}

impl<R, P, S, TQ, Q, QTQ, ITQ>
    FrozenReplayInvocationJournalV1<R, P, S, TQ, Q, QTQ, ITQ>
{
    pub const fn manifest(&self) -> ReplayInvocationJournalManifestV1 {
        self.manifest
    }

    pub const fn reason(&self) -> ReplayInvocationJournalAmbiguityReasonV1 {
        self.reason
    }

    pub const fn expected_store(&self) -> ExpectedReplayInvocationJournalStoreProfileV1 {
        self.expected_store
    }

    pub const fn may_create_fresh_replay_attempt(&self) -> bool {
        false
    }

    pub const fn external_effect_may_have_occurred(&self) -> bool {
        false
    }

    pub fn into_parts(
        self,
    ) -> (
        AuthorizedReplayV1<P, S, TQ>,
        PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    ) {
        (self.authorized_replay, self.prepared_attempt)
    }
}

pub struct RecoverableUnjournaledReplayAttemptV1<P, S, TQ, Q, QTQ, ITQ> {
    authorized_replay: AuthorizedReplayV1<P, S, TQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    receipt: ReplayInvocationJournalReceiptV1,
}

impl<P, S, TQ, Q, QTQ, ITQ>
    RecoverableUnjournaledReplayAttemptV1<P, S, TQ, Q, QTQ, ITQ>
{
    pub const fn receipt(&self) -> ReplayInvocationJournalReceiptV1 {
        self.receipt
    }

    pub fn into_parts(
        self,
    ) -> (
        AuthorizedReplayV1<P, S, TQ>,
        PreparedActuatorEffectAttemptV1<Q, QTQ, ITQ>,
    ) {
        (self.authorized_replay, self.prepared_attempt)
    }
}

pub enum ReplayInvocationJournalOutcomeV1<R, P, S, TQ, Q, QTQ, ITQ> {
    Journaled(DurablyJournaledReplayAttemptV1<R, P, S, TQ, Q, QTQ, ITQ>),
    ProvenNotJournaled(RecoverableUnjournaledReplayAttemptV1<P, S, TQ, Q, QTQ, ITQ>),
    OutcomeUnknown(FrozenReplayInvocationJournalV1<R, P, S, TQ, Q, QTQ, ITQ>),
}

fn validate_journaled_frontier(
    expected: DurableReplayInvocationJournalFrontierV1,
    record: ReplayInvocationJournalRecordCommitment,
    post: DurableReplayInvocationJournalFrontierV1,
) -> Result<(), ReplayInvocationJournalAmbiguityReasonV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(ReplayInvocationJournalAmbiguityReasonV1::JournalGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(record) {
        return Err(ReplayInvocationJournalAmbiguityReasonV1::InvalidJournaledFrontier);
    }
    Ok(())
}

fn absence_frontier_is_compatible(
    expected: DurableReplayInvocationJournalFrontierV1,
    observed: DurableReplayInvocationJournalFrontierV1,
) -> bool {
    if observed.generation < expected.generation {
        return false;
    }
    if observed.generation == expected.generation {
        return observed.head == expected.head;
    }
    true
}

fn freeze_from_prepared<R, P, S, TQ, Q, QTQ, ITQ>(
    prepared: PreparedReplayInvocationJournalV1<P, S, TQ, Q, QTQ, ITQ>,
    reason: ReplayInvocationJournalAmbiguityReasonV1,
) -> ReplayInvocationJournalOutcomeV1<R, P, S, TQ, Q, QTQ, ITQ> {
    ReplayInvocationJournalOutcomeV1::OutcomeUnknown(FrozenReplayInvocationJournalV1 {
        authorized_replay: prepared.authorized_replay,
        prepared_attempt: prepared.prepared_attempt,
        manifest: prepared.manifest,
        expected_store: prepared.expected_store,
        reason,
        _store: PhantomData,
    })
}

fn interpret_receipt<R, P, S, TQ, Q, QTQ, ITQ>(
    prepared: PreparedReplayInvocationJournalV1<P, S, TQ, Q, QTQ, ITQ>,
    receipt: ReplayInvocationJournalReceiptV1,
) -> ReplayInvocationJournalOutcomeV1<R, P, S, TQ, Q, QTQ, ITQ> {
    let manifest = prepared.manifest;
    let latest = manifest.attempt.latest_possible_unix_ms;

    if receipt.schema_version != SSF_SCHEMA_V1 {
        return freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::UnsupportedReceiptSchema,
        );
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1 {
        return freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }
    if receipt.store != prepared.expected_store.descriptor {
        return freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::ReceiptStoreMismatch,
        );
    }
    if receipt.manifest != manifest {
        return freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::ReceiptManifestMismatch,
        );
    }
    if receipt.valid_until > prepared.expected_store.descriptor.valid_until {
        return freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::ReceiptOutlivesStore,
        );
    }
    if receipt.valid_until < latest {
        return freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::ReceiptAlreadyExpired,
        );
    }

    match receipt.disposition {
        ReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => {
            if let Err(reason) =
                validate_journaled_frontier(manifest.expected_frontier, record, post_frontier)
            {
                return freeze_from_prepared(prepared, reason);
            }
            ReplayInvocationJournalOutcomeV1::Journaled(DurablyJournaledReplayAttemptV1 {
                authorized_replay: prepared.authorized_replay,
                prepared_attempt: prepared.prepared_attempt,
                manifest,
                receipt,
                record,
                post_frontier,
                _store: PhantomData,
            })
        }
        ReplayInvocationJournalDispositionV1::ProvenNotJournaled {
            observed_frontier,
            ..
        } => {
            if !absence_frontier_is_compatible(manifest.expected_frontier, observed_frontier) {
                return freeze_from_prepared(
                    prepared,
                    ReplayInvocationJournalAmbiguityReasonV1::InvalidProvenNotJournaledFrontier,
                );
            }
            ReplayInvocationJournalOutcomeV1::ProvenNotJournaled(
                RecoverableUnjournaledReplayAttemptV1 {
                    authorized_replay: prepared.authorized_replay,
                    prepared_attempt: prepared.prepared_attempt,
                    receipt,
                },
            )
        }
        ReplayInvocationJournalDispositionV1::AttemptIdConflict { .. } => freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::AttemptIdConflict,
        ),
        ReplayInvocationJournalDispositionV1::ReplayAuthorizationAlreadyConsumed { .. } => {
            freeze_from_prepared(
                prepared,
                ReplayInvocationJournalAmbiguityReasonV1::ReplayAuthorizationAlreadyConsumed,
            )
        }
        ReplayInvocationJournalDispositionV1::PriorAttemptAlreadyReplayed { .. } => {
            freeze_from_prepared(
                prepared,
                ReplayInvocationJournalAmbiguityReasonV1::PriorAttemptAlreadyReplayed,
            )
        }
        ReplayInvocationJournalDispositionV1::OutcomeUnknown => freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

pub fn journal_replay_attempt<R, P, S, TQ, Q, QTQ, ITQ>(
    prepared: PreparedReplayInvocationJournalV1<P, S, TQ, Q, QTQ, ITQ>,
    store: &R,
) -> Result<
    ReplayInvocationJournalOutcomeV1<R, P, S, TQ, Q, QTQ, ITQ>,
    PreparedReplayInvocationJournalV1<P, S, TQ, Q, QTQ, ITQ>,
>
where
    R: ReplayInvocationJournalStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor {
        return Err(prepared);
    }

    let receipt = match store.journal_replay(&prepared.manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Ok(freeze_from_prepared(
                prepared,
                ReplayInvocationJournalAmbiguityReasonV1::StoreErrorAfterJournalBoundary,
            ));
        }
    };

    if store.descriptor() != prepared.expected_store.descriptor {
        return Ok(freeze_from_prepared(
            prepared,
            ReplayInvocationJournalAmbiguityReasonV1::StoreDescriptorChangedAfter,
        ));
    }

    Ok(interpret_receipt::<R, P, S, TQ, Q, QTQ, ITQ>(
        prepared, receipt,
    ))
}

/// Read-only restart evidence recovered by the unique prior attempt ID.
pub struct RecoveredReplayInvocationJournalV1<R> {
    receipt: ReplayInvocationJournalReceiptV1,
    _store: PhantomData<fn() -> R>,
}

impl<R> RecoveredReplayInvocationJournalV1<R> {
    pub const fn receipt(&self) -> ReplayInvocationJournalReceiptV1 {
        self.receipt
    }

    pub const fn external_effect_attempted_by_recovery(&self) -> bool {
        false
    }
}

pub fn recover_replay_by_prior_attempt<R>(
    store: &R,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    prior_attempt_id: ActuatorInvocationAttemptId,
) -> Result<RecoveredReplayInvocationJournalV1<R>, ReplayInvocationJournalAmbiguityReasonV1>
where
    R: ReplayInvocationJournalStoreV1,
{
    if store.descriptor() != expected_store.descriptor {
        return Err(
            ReplayInvocationJournalAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = store
        .reconcile_prior_attempt(prior_attempt_id)
        .map_err(|_| ReplayInvocationJournalAmbiguityReasonV1::StoreErrorDuringReconciliation)?;

    if store.descriptor() != expected_store.descriptor {
        return Err(ReplayInvocationJournalAmbiguityReasonV1::StoreDescriptorChangedAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(ReplayInvocationJournalAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1 {
        return Err(ReplayInvocationJournalAmbiguityReasonV1::UnsupportedManifestSchema);
    }
    if receipt.store != expected_store.descriptor {
        return Err(ReplayInvocationJournalAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest.prior_attempt_id() != prior_attempt_id {
        return Err(ReplayInvocationJournalAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.descriptor.valid_until {
        return Err(ReplayInvocationJournalAmbiguityReasonV1::ReceiptOutlivesStore);
    }

    match receipt.disposition {
        ReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => {
            validate_journaled_frontier(
                receipt.manifest.expected_frontier,
                record,
                post_frontier,
            )?;
            Ok(RecoveredReplayInvocationJournalV1 {
                receipt,
                _store: PhantomData,
            })
        }
        ReplayInvocationJournalDispositionV1::ProvenNotJournaled {
            observed_frontier,
            ..
        } => {
            if !absence_frontier_is_compatible(
                receipt.manifest.expected_frontier,
                observed_frontier,
            ) {
                return Err(
                    ReplayInvocationJournalAmbiguityReasonV1::InvalidProvenNotJournaledFrontier,
                );
            }
            Ok(RecoveredReplayInvocationJournalV1 {
                receipt,
                _store: PhantomData,
            })
        }
        ReplayInvocationJournalDispositionV1::AttemptIdConflict { .. } => {
            Err(ReplayInvocationJournalAmbiguityReasonV1::AttemptIdConflict)
        }
        ReplayInvocationJournalDispositionV1::ReplayAuthorizationAlreadyConsumed { .. } => Err(
            ReplayInvocationJournalAmbiguityReasonV1::ReplayAuthorizationAlreadyConsumed,
        ),
        ReplayInvocationJournalDispositionV1::PriorAttemptAlreadyReplayed { .. } => Err(
            ReplayInvocationJournalAmbiguityReasonV1::PriorAttemptAlreadyReplayed,
        ),
        ReplayInvocationJournalDispositionV1::OutcomeUnknown => Err(
            ReplayInvocationJournalAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    #[test]
    fn replay_attempt_must_use_a_new_attempt_id() {
        let prior = ActuatorInvocationAttemptId::from_bytes(bytes(1));
        let next = ActuatorInvocationAttemptId::from_bytes(bytes(2));
        assert_ne!(prior, next);
    }

    #[test]
    fn frontier_requires_exact_plus_one_progression() {
        let expected = DurableReplayInvocationJournalFrontierV1 {
            generation: DurableReplayInvocationJournalGeneration::new(7),
            head: None,
        };
        let record = ReplayInvocationJournalRecordCommitment::from_bytes(bytes(3));
        let valid = DurableReplayInvocationJournalFrontierV1 {
            generation: DurableReplayInvocationJournalGeneration::new(8),
            head: Some(record),
        };
        assert_eq!(validate_journaled_frontier(expected, record, valid), Ok(()));
    }
}

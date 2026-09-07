// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Crash-safe journal for one canonical replay authorization.
//!
//! This crate consumes `AuthorizedCanonicalReplayV1` plus a newly prepared,
//! freshly qualified actuator attempt. It reuses the established replay-journal
//! store identity/frontier/record vocabulary, but defines a distinct canonical
//! manifest and receipt domain. Legacy and canonical replay authorizations are
//! never interchangeable.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    ActuatorInvocationAttemptId, ActuatorInvocationAttemptManifestV1,
    PreparedActuatorEffectAttemptV1,
};
use mycelix_ssf_canonical_effect_outcome_history::CanonicalInvocationRecordV1;
use mycelix_ssf_canonical_replay_policy::{
    AuthorizedCanonicalReplayV1, CanonicalReplayAuthorizationCommitment,
    CanonicalReplayPolicyReceiptV1, CanonicalReplayPolicySubjectV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_replay_aware_invocation_journal::{
    DurableReplayInvocationJournalFrontierV1, ExpectedDurableReplayInvocationJournalFrontierV1,
    ExpectedReplayInvocationJournalStoreProfileV1, ReplayInvocationJournalAbsenceCommitment,
    ReplayInvocationJournalRecordCommitment, ReplayInvocationJournalStoreDescriptorV1,
    ReplayInvocationJournalTimeBasisV1,
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

digest_type!(CanonicalReplayInvocationJournalManifestCommitment);
digest_type!(CanonicalReplayInvocationJournalReceiptCommitment);

/// Copyable audit binding extracted from the non-copyable canonical replay
/// authorization. This is evidence of what was authorized, not authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayAuthorizationBindingV1 {
    pub subject: CanonicalReplayPolicySubjectV1,
    pub policy_receipt: CanonicalReplayPolicyReceiptV1,
    pub authorization: CanonicalReplayAuthorizationCommitment,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayInvocationJournalManifestV1 {
    pub schema_version: u16,
    pub replay: CanonicalReplayAuthorizationBindingV1,
    pub prior_invocation_record: CanonicalInvocationRecordV1,
    pub attempt: ActuatorInvocationAttemptManifestV1,
    pub expected_store: ReplayInvocationJournalStoreDescriptorV1,
    pub expected_frontier: DurableReplayInvocationJournalFrontierV1,
}

impl CanonicalReplayInvocationJournalManifestV1 {
    pub const fn attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.attempt.attempt_id
    }

    pub const fn prior_attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.replay.subject.evidence_subject.prior_attempt_id
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReplayInvocationJournalDispositionV1 {
    Journaled {
        record: ReplayInvocationJournalRecordCommitment,
        post_frontier: DurableReplayInvocationJournalFrontierV1,
    },
    ProvenNotJournaled {
        observed_frontier: DurableReplayInvocationJournalFrontierV1,
        absence_evidence: ReplayInvocationJournalAbsenceCommitment,
    },
    AttemptIdConflict {
        existing_manifest: CanonicalReplayInvocationJournalManifestCommitment,
    },
    CanonicalReplayAuthorizationAlreadyConsumed {
        existing_record: ReplayInvocationJournalRecordCommitment,
    },
    PriorAttemptAlreadyReplayed {
        existing_record: ReplayInvocationJournalRecordCommitment,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayInvocationJournalReceiptV1 {
    pub schema_version: u16,
    pub store: ReplayInvocationJournalStoreDescriptorV1,
    pub manifest: CanonicalReplayInvocationJournalManifestV1,
    pub disposition: CanonicalReplayInvocationJournalDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: CanonicalReplayInvocationJournalReceiptCommitment,
}

/// Canonical replay journal contract.
///
/// Implementations MUST enforce all three durable uniqueness keys:
///
/// - first-seen new attempt ID permanently binds its exact manifest;
/// - one canonical replay authorization commitment produces at most one replay;
/// - one prior attempt produces at most one replay record in v0.1.
///
/// `ProvenNotJournaled` is final for that attempt ID. Reconciliation methods are
/// read-only and must never invoke an actuator.
pub trait CanonicalReplayInvocationJournalStoreV1 {
    type Error;

    fn descriptor(&self) -> ReplayInvocationJournalStoreDescriptorV1;

    fn journal_canonical_replay(
        &self,
        manifest: &CanonicalReplayInvocationJournalManifestV1,
    ) -> Result<CanonicalReplayInvocationJournalReceiptV1, Self::Error>;

    fn reconcile_canonical_replay(
        &self,
        manifest: &CanonicalReplayInvocationJournalManifestV1,
    ) -> Result<CanonicalReplayInvocationJournalReceiptV1, Self::Error>;

    /// Safe in v0.1 because one prior attempt may have at most one replay.
    fn reconcile_prior_attempt(
        &self,
        prior_attempt_id: ActuatorInvocationAttemptId,
    ) -> Result<CanonicalReplayInvocationJournalReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReplayInvocationPreparationErrorV1 {
    UnsupportedStoreTimeBasis,
    UnsupportedAttemptSchema,
    AuthorizationStableIdentityIncoherent,
    PriorEvidenceStableIdentityIncoherent,
    StableEffectIdentityMismatch,
    PriorInvocationRecordMismatch,
    ReusedPriorAttemptId,
    AttemptTimeRegressedBeforePolicyDecision,
    CanonicalReplayAuthorizationAlreadyExpired,
    PreparedAttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
    StoreAlreadyExpired,
}

pub struct PreparedCanonicalReplayInvocationJournalV1<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ> {
    authorized_replay: AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ>,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    manifest: CanonicalReplayInvocationJournalManifestV1,
}

impl<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
    PreparedCanonicalReplayInvocationJournalV1<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
{
    pub const fn authorized_replay(
        &self,
    ) -> &AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ> {
        &self.authorized_replay
    }

    pub const fn prepared_attempt(&self) -> &PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ> {
        &self.prepared_attempt
    }

    pub const fn expected_store(&self) -> ExpectedReplayInvocationJournalStoreProfileV1 {
        self.expected_store
    }

    pub const fn manifest(&self) -> CanonicalReplayInvocationJournalManifestV1 {
        self.manifest
    }
}

pub struct CanonicalReplayInvocationPreparationFailureV1<
    P,
    EQ,
    S,
    HTQ,
    QTQ,
    PTQ,
    AQ,
    AQTQ,
    AITQ,
> {
    authorized_replay: AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ>,
    error: CanonicalReplayInvocationPreparationErrorV1,
}

impl<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
    CanonicalReplayInvocationPreparationFailureV1<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
{
    pub const fn error(&self) -> CanonicalReplayInvocationPreparationErrorV1 {
        self.error
    }

    pub fn into_parts(
        self,
    ) -> (
        AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ>,
        PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ>,
    ) {
        (self.authorized_replay, self.prepared_attempt)
    }
}

#[allow(clippy::too_many_arguments)]
pub fn prepare_canonical_replay_invocation_journal<
    P,
    EQ,
    S,
    HTQ,
    QTQ,
    PTQ,
    AQ,
    AQTQ,
    AITQ,
>(
    authorized_replay: AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ>,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    expected_frontier: ExpectedDurableReplayInvocationJournalFrontierV1,
) -> Result<
    PreparedCanonicalReplayInvocationJournalV1<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>,
    CanonicalReplayInvocationPreparationFailureV1<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>,
> {
    let attempt = prepared_attempt.manifest();
    let replay_subject = authorized_replay.subject();
    let evidence_subject = replay_subject.evidence_subject;
    let latest = attempt.latest_possible_unix_ms;
    let prior_invocation_record = evidence_subject
        .latest_entry
        .manifest
        .subject
        .invocation_record;

    let error = if expected_store.descriptor().time_basis
        != ReplayInvocationJournalTimeBasisV1::UnixMillisecondsUtc
    {
        Some(CanonicalReplayInvocationPreparationErrorV1::UnsupportedStoreTimeBasis)
    } else if attempt.schema_version != SSF_SCHEMA_V1 {
        Some(CanonicalReplayInvocationPreparationErrorV1::UnsupportedAttemptSchema)
    } else if replay_subject.stable_effect_identity != evidence_subject.stable_effect_identity {
        Some(CanonicalReplayInvocationPreparationErrorV1::AuthorizationStableIdentityIncoherent)
    } else if evidence_subject.prior_effect_subject.stable_identity()
        != evidence_subject.stable_effect_identity
    {
        Some(CanonicalReplayInvocationPreparationErrorV1::PriorEvidenceStableIdentityIncoherent)
    } else if attempt.subject.stable_identity() != replay_subject.stable_effect_identity {
        Some(CanonicalReplayInvocationPreparationErrorV1::StableEffectIdentityMismatch)
    } else if evidence_subject.latest_entry.manifest.subject.evidence.attempt().attempt_id
        != evidence_subject.prior_attempt_id
    {
        Some(CanonicalReplayInvocationPreparationErrorV1::PriorInvocationRecordMismatch)
    } else if attempt.attempt_id == evidence_subject.prior_attempt_id {
        Some(CanonicalReplayInvocationPreparationErrorV1::ReusedPriorAttemptId)
    } else if latest < replay_subject.policy_latest_possible_unix_ms {
        Some(CanonicalReplayInvocationPreparationErrorV1::AttemptTimeRegressedBeforePolicyDecision)
    } else if authorized_replay.valid_until() < latest {
        Some(CanonicalReplayInvocationPreparationErrorV1::CanonicalReplayAuthorizationAlreadyExpired)
    } else if attempt.attempt_valid_until < latest {
        Some(CanonicalReplayInvocationPreparationErrorV1::PreparedAttemptAlreadyExpired)
    } else if attempt.subject.actuator.valid_until < latest {
        Some(CanonicalReplayInvocationPreparationErrorV1::ActuatorGenerationAlreadyExpired)
    } else if expected_store.descriptor().valid_until < latest {
        Some(CanonicalReplayInvocationPreparationErrorV1::StoreAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(CanonicalReplayInvocationPreparationFailureV1 {
            authorized_replay,
            prepared_attempt,
            error,
        });
    }

    let replay = CanonicalReplayAuthorizationBindingV1 {
        subject: replay_subject,
        policy_receipt: authorized_replay.receipt(),
        authorization: authorized_replay.authorization(),
        valid_until: authorized_replay.valid_until(),
    };
    let manifest = CanonicalReplayInvocationJournalManifestV1 {
        schema_version: SSF_SCHEMA_V1,
        replay,
        prior_invocation_record,
        attempt,
        expected_store: expected_store.descriptor(),
        expected_frontier: expected_frontier.frontier(),
    };

    Ok(PreparedCanonicalReplayInvocationJournalV1 {
        authorized_replay,
        prepared_attempt,
        expected_store,
        manifest,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayInvocationJournalAmbiguityReasonV1 {
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
    CanonicalReplayAuthorizationAlreadyConsumed,
    PriorAttemptAlreadyReplayed,
    StoreReportedOutcomeUnknown,
}

fn validate_journaled_frontier(
    expected: DurableReplayInvocationJournalFrontierV1,
    record: ReplayInvocationJournalRecordCommitment,
    post: DurableReplayInvocationJournalFrontierV1,
) -> Result<(), CanonicalReplayInvocationJournalAmbiguityReasonV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(CanonicalReplayInvocationJournalAmbiguityReasonV1::JournalGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(record) {
        return Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::InvalidJournaledFrontier);
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

pub struct DurablyJournaledCanonicalReplayAttemptV1<
    R,
    P,
    EQ,
    S,
    HTQ,
    QTQ,
    PTQ,
    AQ,
    AQTQ,
    AITQ,
> {
    authorized_replay: AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ>,
    manifest: CanonicalReplayInvocationJournalManifestV1,
    receipt: CanonicalReplayInvocationJournalReceiptV1,
    record: ReplayInvocationJournalRecordCommitment,
    post_frontier: DurableReplayInvocationJournalFrontierV1,
    _store: PhantomData<fn() -> R>,
}

impl<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
    DurablyJournaledCanonicalReplayAttemptV1<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
{
    pub const fn authorized_replay(
        &self,
    ) -> &AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ> {
        &self.authorized_replay
    }

    pub const fn prepared_attempt(&self) -> &PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ> {
        &self.prepared_attempt
    }

    pub const fn manifest(&self) -> CanonicalReplayInvocationJournalManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> CanonicalReplayInvocationJournalReceiptV1 {
        self.receipt
    }

    pub const fn record(&self) -> ReplayInvocationJournalRecordCommitment {
        self.record
    }

    pub const fn post_frontier(&self) -> DurableReplayInvocationJournalFrontierV1 {
        self.post_frontier
    }

    pub const fn eligible_for_fresh_activation(&self) -> bool {
        true
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

/// Final non-persistence returns the replay authorization but intentionally
/// discards the old prepared attempt. A caller must prepare a new attempt with a
/// new ID and fresh qualification/time before another journal write is possible.
pub struct RecoverableCanonicalReplayAuthorizationV1<P, EQ, S, HTQ, QTQ, PTQ> {
    authorized_replay: AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ>,
    receipt: CanonicalReplayInvocationJournalReceiptV1,
}

impl<P, EQ, S, HTQ, QTQ, PTQ>
    RecoverableCanonicalReplayAuthorizationV1<P, EQ, S, HTQ, QTQ, PTQ>
{
    pub const fn receipt(&self) -> CanonicalReplayInvocationJournalReceiptV1 {
        self.receipt
    }

    pub fn into_authorization(self) -> AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ> {
        self.authorized_replay
    }

    pub const fn requires_new_attempt_preparation(&self) -> bool {
        true
    }
}

pub struct FrozenCanonicalReplayInvocationJournalV1<
    R,
    P,
    EQ,
    S,
    HTQ,
    QTQ,
    PTQ,
    AQ,
    AQTQ,
    AITQ,
> {
    authorized_replay: AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ>,
    prepared_attempt: PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ>,
    manifest: CanonicalReplayInvocationJournalManifestV1,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    reason: CanonicalReplayInvocationJournalAmbiguityReasonV1,
    _store: PhantomData<fn() -> R>,
}

impl<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
    FrozenCanonicalReplayInvocationJournalV1<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>
{
    pub const fn reason(&self) -> CanonicalReplayInvocationJournalAmbiguityReasonV1 {
        self.reason
    }

    pub const fn manifest(&self) -> CanonicalReplayInvocationJournalManifestV1 {
        self.manifest
    }

    pub const fn expected_store(&self) -> ExpectedReplayInvocationJournalStoreProfileV1 {
        self.expected_store
    }

    pub const fn may_prepare_new_attempt(&self) -> bool {
        false
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub fn authorization(&self) -> &AuthorizedCanonicalReplayV1<P, EQ, S, HTQ, QTQ, PTQ> {
        &self.authorized_replay
    }

    pub fn prepared_attempt(&self) -> &PreparedActuatorEffectAttemptV1<AQ, AQTQ, AITQ> {
        &self.prepared_attempt
    }
}

pub enum CanonicalReplayInvocationJournalOutcomeV1<
    R,
    P,
    EQ,
    S,
    HTQ,
    QTQ,
    PTQ,
    AQ,
    AQTQ,
    AITQ,
> {
    Journaled(
        DurablyJournaledCanonicalReplayAttemptV1<
            R,
            P,
            EQ,
            S,
            HTQ,
            QTQ,
            PTQ,
            AQ,
            AQTQ,
            AITQ,
        >,
    ),
    ProvenNotJournaled(RecoverableCanonicalReplayAuthorizationV1<P, EQ, S, HTQ, QTQ, PTQ>),
    OutcomeUnknown(
        FrozenCanonicalReplayInvocationJournalV1<
            R,
            P,
            EQ,
            S,
            HTQ,
            QTQ,
            PTQ,
            AQ,
            AQTQ,
            AITQ,
        >,
    ),
}

fn freeze_from_prepared<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>(
    prepared: PreparedCanonicalReplayInvocationJournalV1<
        P,
        EQ,
        S,
        HTQ,
        QTQ,
        PTQ,
        AQ,
        AQTQ,
        AITQ,
    >,
    reason: CanonicalReplayInvocationJournalAmbiguityReasonV1,
) -> CanonicalReplayInvocationJournalOutcomeV1<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ> {
    CanonicalReplayInvocationJournalOutcomeV1::OutcomeUnknown(
        FrozenCanonicalReplayInvocationJournalV1 {
            authorized_replay: prepared.authorized_replay,
            prepared_attempt: prepared.prepared_attempt,
            manifest: prepared.manifest,
            expected_store: prepared.expected_store,
            reason,
            _store: PhantomData,
        },
    )
}

fn interpret_receipt<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>(
    prepared: PreparedCanonicalReplayInvocationJournalV1<
        P,
        EQ,
        S,
        HTQ,
        QTQ,
        PTQ,
        AQ,
        AQTQ,
        AITQ,
    >,
    receipt: CanonicalReplayInvocationJournalReceiptV1,
) -> CanonicalReplayInvocationJournalOutcomeV1<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ> {
    let manifest = prepared.manifest;
    let latest = manifest.attempt.latest_possible_unix_ms;

    if receipt.schema_version != SSF_SCHEMA_V1 {
        return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::UnsupportedReceiptSchema,
        );
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1 {
        return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::UnsupportedManifestSchema,
        );
    }
    if receipt.store != prepared.expected_store.descriptor() {
        return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::ReceiptStoreMismatch,
        );
    }
    if receipt.manifest != manifest {
        return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::ReceiptManifestMismatch,
        );
    }
    if receipt.valid_until > prepared.expected_store.descriptor().valid_until {
        return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::ReceiptOutlivesStore,
        );
    }
    if receipt.valid_until < latest {
        return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::ReceiptAlreadyExpired,
        );
    }

    match receipt.disposition {
        CanonicalReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => {
            if let Err(reason) =
                validate_journaled_frontier(manifest.expected_frontier, record, post_frontier)
            {
                return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(prepared, reason);
            }
            CanonicalReplayInvocationJournalOutcomeV1::Journaled(
                DurablyJournaledCanonicalReplayAttemptV1 {
                    authorized_replay: prepared.authorized_replay,
                    prepared_attempt: prepared.prepared_attempt,
                    manifest,
                    receipt,
                    record,
                    post_frontier,
                    _store: PhantomData,
                },
            )
        }
        CanonicalReplayInvocationJournalDispositionV1::ProvenNotJournaled {
            observed_frontier,
            ..
        } => {
            if !absence_frontier_is_compatible(manifest.expected_frontier, observed_frontier) {
                return freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
                    prepared,
                    CanonicalReplayInvocationJournalAmbiguityReasonV1::InvalidProvenNotJournaledFrontier,
                );
            }
            CanonicalReplayInvocationJournalOutcomeV1::ProvenNotJournaled(
                RecoverableCanonicalReplayAuthorizationV1 {
                    authorized_replay: prepared.authorized_replay,
                    receipt,
                },
            )
        }
        CanonicalReplayInvocationJournalDispositionV1::AttemptIdConflict { .. } => {
            freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
                prepared,
                CanonicalReplayInvocationJournalAmbiguityReasonV1::AttemptIdConflict,
            )
        }
        CanonicalReplayInvocationJournalDispositionV1::CanonicalReplayAuthorizationAlreadyConsumed {
            ..
        } => freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::CanonicalReplayAuthorizationAlreadyConsumed,
        ),
        CanonicalReplayInvocationJournalDispositionV1::PriorAttemptAlreadyReplayed { .. } => {
            freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
                prepared,
                CanonicalReplayInvocationJournalAmbiguityReasonV1::PriorAttemptAlreadyReplayed,
            )
        }
        CanonicalReplayInvocationJournalDispositionV1::OutcomeUnknown => {
            freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
                prepared,
                CanonicalReplayInvocationJournalAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            )
        }
    }
}

pub fn journal_canonical_replay_attempt<
    R,
    P,
    EQ,
    S,
    HTQ,
    QTQ,
    PTQ,
    AQ,
    AQTQ,
    AITQ,
>(
    prepared: PreparedCanonicalReplayInvocationJournalV1<
        P,
        EQ,
        S,
        HTQ,
        QTQ,
        PTQ,
        AQ,
        AQTQ,
        AITQ,
    >,
    store: &R,
) -> Result<
    CanonicalReplayInvocationJournalOutcomeV1<
        R,
        P,
        EQ,
        S,
        HTQ,
        QTQ,
        PTQ,
        AQ,
        AQTQ,
        AITQ,
    >,
    PreparedCanonicalReplayInvocationJournalV1<P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ>,
>
where
    R: CanonicalReplayInvocationJournalStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor() {
        return Err(prepared);
    }

    let receipt = match store.journal_canonical_replay(&prepared.manifest) {
        Ok(value) => value,
        Err(_) => {
            return Ok(freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
                prepared,
                CanonicalReplayInvocationJournalAmbiguityReasonV1::StoreErrorAfterJournalBoundary,
            ));
        }
    };

    if store.descriptor() != prepared.expected_store.descriptor() {
        return Ok(freeze_from_prepared::<R, _, _, _, _, _, _, _, _, _>(
            prepared,
            CanonicalReplayInvocationJournalAmbiguityReasonV1::StoreDescriptorChangedAfter,
        ));
    }

    Ok(interpret_receipt::<R, _, _, _, _, _, _, _, _, _>(
        prepared, receipt,
    ))
}

/// Read-only restart evidence keyed by the unique prior attempt ID. This does
/// not reconstruct `AuthorizedCanonicalReplayV1` or a prepared actuator attempt.
pub struct RecoveredCanonicalReplayInvocationJournalV1<R> {
    receipt: CanonicalReplayInvocationJournalReceiptV1,
    _store: PhantomData<fn() -> R>,
}

impl<R> RecoveredCanonicalReplayInvocationJournalV1<R> {
    pub const fn receipt(&self) -> CanonicalReplayInvocationJournalReceiptV1 {
        self.receipt
    }

    pub const fn external_effect_attempted_by_recovery(&self) -> bool {
        false
    }
}

pub fn recover_canonical_replay_by_prior_attempt<R>(
    store: &R,
    expected_store: ExpectedReplayInvocationJournalStoreProfileV1,
    prior_attempt_id: ActuatorInvocationAttemptId,
) -> Result<
    RecoveredCanonicalReplayInvocationJournalV1<R>,
    CanonicalReplayInvocationJournalAmbiguityReasonV1,
>
where
    R: CanonicalReplayInvocationJournalStoreV1,
{
    if store.descriptor() != expected_store.descriptor() {
        return Err(
            CanonicalReplayInvocationJournalAmbiguityReasonV1::StoreDescriptorMismatchBeforeReconciliation,
        );
    }

    let receipt = store
        .reconcile_prior_attempt(prior_attempt_id)
        .map_err(|_| CanonicalReplayInvocationJournalAmbiguityReasonV1::StoreErrorDuringReconciliation)?;

    if store.descriptor() != expected_store.descriptor() {
        return Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::StoreDescriptorChangedAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1 {
        return Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::UnsupportedManifestSchema);
    }
    if receipt.store != expected_store.descriptor() {
        return Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest.prior_attempt_id() != prior_attempt_id {
        return Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > expected_store.descriptor().valid_until {
        return Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::ReceiptOutlivesStore);
    }

    match receipt.disposition {
        CanonicalReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => {
            validate_journaled_frontier(
                receipt.manifest.expected_frontier,
                record,
                post_frontier,
            )?;
            Ok(RecoveredCanonicalReplayInvocationJournalV1 {
                receipt,
                _store: PhantomData,
            })
        }
        CanonicalReplayInvocationJournalDispositionV1::ProvenNotJournaled {
            observed_frontier,
            ..
        } => {
            if !absence_frontier_is_compatible(
                receipt.manifest.expected_frontier,
                observed_frontier,
            ) {
                return Err(
                    CanonicalReplayInvocationJournalAmbiguityReasonV1::InvalidProvenNotJournaledFrontier,
                );
            }
            Ok(RecoveredCanonicalReplayInvocationJournalV1 {
                receipt,
                _store: PhantomData,
            })
        }
        CanonicalReplayInvocationJournalDispositionV1::AttemptIdConflict { .. } => {
            Err(CanonicalReplayInvocationJournalAmbiguityReasonV1::AttemptIdConflict)
        }
        CanonicalReplayInvocationJournalDispositionV1::CanonicalReplayAuthorizationAlreadyConsumed {
            ..
        } => Err(
            CanonicalReplayInvocationJournalAmbiguityReasonV1::CanonicalReplayAuthorizationAlreadyConsumed,
        ),
        CanonicalReplayInvocationJournalDispositionV1::PriorAttemptAlreadyReplayed { .. } => Err(
            CanonicalReplayInvocationJournalAmbiguityReasonV1::PriorAttemptAlreadyReplayed,
        ),
        CanonicalReplayInvocationJournalDispositionV1::OutcomeUnknown => Err(
            CanonicalReplayInvocationJournalAmbiguityReasonV1::StoreReportedOutcomeUnknown,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(value: u8) -> [u8; 32] {
        [value; 32]
    }

    #[test]
    fn canonical_and_legacy_manifest_domains_are_distinct() {
        assert_ne!(
            core::any::type_name::<CanonicalReplayInvocationJournalManifestCommitment>(),
            core::any::type_name::<
                mycelix_ssf_replay_aware_invocation_journal::ReplayInvocationJournalManifestCommitment,
            >()
        );
    }

    #[test]
    fn replay_attempt_must_use_new_attempt_id() {
        let prior = ActuatorInvocationAttemptId::from_bytes(bytes(1));
        let next = ActuatorInvocationAttemptId::from_bytes(bytes(2));
        assert_ne!(prior, next);
    }

    #[test]
    fn frontier_requires_exact_plus_one_progression() {
        let expected = DurableReplayInvocationJournalFrontierV1 {
            generation:
                mycelix_ssf_replay_aware_invocation_journal::DurableReplayInvocationJournalGeneration::new(7),
            head: None,
        };
        let record = ReplayInvocationJournalRecordCommitment::from_bytes(bytes(3));
        let valid = DurableReplayInvocationJournalFrontierV1 {
            generation:
                mycelix_ssf_replay_aware_invocation_journal::DurableReplayInvocationJournalGeneration::new(8),
            head: Some(record),
        };
        assert_eq!(validate_journaled_frontier(expected, record, valid), Ok(()));
    }
}
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Append-only durable outcome history for one exact journaled SSF actuator
//! invocation attempt.
//!
//! Effect reality and recording durability are different state machines.
//! Failure to persist an already validated actuator outcome never rewrites that
//! outcome into non-application. `OutcomeUnknown` may later refine to a
//! terminal state; `Confirmed` and `ProvenNotApplied` are terminal.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    recovery_policy_for, ActuatorEffectAmbiguityReasonV1, ActuatorEffectOutcomeCommitment,
    ActuatorEffectReceiptCommitment, ActuatorNonApplicationEvidenceCommitment,
    EffectRecoveryPolicyV1, ValidatedActuatorEffectDispositionV1,
    ValidatedActuatorEffectOutcomeV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_durable_actuator_invocation_attempt::{
    DurablyJournaledActuatorEffectAttemptV1, InvocationAttemptJournalManifestV1,
    InvocationAttemptJournalReceiptCommitment, InvocationAttemptJournalRecordCommitment,
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

digest_type!(EffectOutcomeObservationId);
digest_type!(EffectOutcomeRecordCommitment);
digest_type!(EffectOutcomeStoreReceiptCommitment);
digest_type!(EffectOutcomeAbsenceCommitment);
digest_type!(EffectOutcomeObservationManifestCommitment);
digest_type!(EffectOutcomeStoreIdentityCommitment);
digest_type!(EffectOutcomeStorePolicyCommitment);
digest_type!(EffectOutcomeHistoryHeadReceiptCommitment);

generation_type!(EffectOutcomeStoreGeneration);
generation_type!(EffectOutcomeHistoryGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectOutcomeStoreTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EffectOutcomeStoreDescriptorV1 {
    pub stable_identity: EffectOutcomeStoreIdentityCommitment,
    pub policy: EffectOutcomeStorePolicyCommitment,
    pub generation: EffectOutcomeStoreGeneration,
    pub time_basis: EffectOutcomeStoreTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedEffectOutcomeStoreProfileV1 {
    descriptor: EffectOutcomeStoreDescriptorV1,
}

impl ExpectedEffectOutcomeStoreProfileV1 {
    pub const fn from_trusted_configuration(descriptor: EffectOutcomeStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> EffectOutcomeStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectOutcomeTerminalV1 {
    Confirmed,
    ProvenNotApplied,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectOutcomeHistoryHeadError {
    GenesisHasRecord,
    GenesisIsTerminal,
    NonGenesisMissingRecord,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EffectOutcomeHistoryHeadV1 {
    journal_record: InvocationAttemptJournalRecordCommitment,
    generation: EffectOutcomeHistoryGeneration,
    head: Option<EffectOutcomeRecordCommitment>,
    terminal: Option<EffectOutcomeTerminalV1>,
}

impl EffectOutcomeHistoryHeadV1 {
    pub fn from_trusted_state(
        journal_record: InvocationAttemptJournalRecordCommitment,
        generation: EffectOutcomeHistoryGeneration,
        head: Option<EffectOutcomeRecordCommitment>,
        terminal: Option<EffectOutcomeTerminalV1>,
    ) -> Result<Self, EffectOutcomeHistoryHeadError> {
        match (generation.get(), head, terminal) {
            (0, Some(_), _) => return Err(EffectOutcomeHistoryHeadError::GenesisHasRecord),
            (0, None, Some(_)) => return Err(EffectOutcomeHistoryHeadError::GenesisIsTerminal),
            (0, None, None) => {}
            (_, None, _) => return Err(EffectOutcomeHistoryHeadError::NonGenesisMissingRecord),
            (_, Some(_), _) => {}
        }
        Ok(Self {
            journal_record,
            generation,
            head,
            terminal,
        })
    }

    pub const fn journal_record(&self) -> InvocationAttemptJournalRecordCommitment {
        self.journal_record
    }
    pub const fn generation(&self) -> EffectOutcomeHistoryGeneration {
        self.generation
    }
    pub const fn head(&self) -> Option<EffectOutcomeRecordCommitment> {
        self.head
    }
    pub const fn terminal(&self) -> Option<EffectOutcomeTerminalV1> {
        self.terminal
    }
    pub const fn permits_append(&self) -> bool {
        self.terminal.is_none()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedEffectOutcomeHistoryHeadV1 {
    head: EffectOutcomeHistoryHeadV1,
}

impl ExpectedEffectOutcomeHistoryHeadV1 {
    pub const fn from_trusted_state(head: EffectOutcomeHistoryHeadV1) -> Self {
        Self { head }
    }
    pub const fn head(&self) -> EffectOutcomeHistoryHeadV1 {
        self.head
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectOutcomeStateV1 {
    Confirmed {
        outcome: ActuatorEffectOutcomeCommitment,
        actuator_receipt: ActuatorEffectReceiptCommitment,
    },
    ProvenNotApplied {
        evidence: ActuatorNonApplicationEvidenceCommitment,
        actuator_receipt: ActuatorEffectReceiptCommitment,
    },
    OutcomeUnknown {
        reason: ActuatorEffectAmbiguityReasonV1,
        actuator_receipt: Option<ActuatorEffectReceiptCommitment>,
        recovery_policy: EffectRecoveryPolicyV1,
    },
}

impl EffectOutcomeStateV1 {
    pub const fn terminal(self) -> Option<EffectOutcomeTerminalV1> {
        match self {
            Self::Confirmed { .. } => Some(EffectOutcomeTerminalV1::Confirmed),
            Self::ProvenNotApplied { .. } => Some(EffectOutcomeTerminalV1::ProvenNotApplied),
            Self::OutcomeUnknown { .. } => None,
        }
    }

    pub const fn replay_authority_created(self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectOutcomeObservationSubjectV1 {
    pub journal_record: InvocationAttemptJournalRecordCommitment,
    pub journal_manifest: InvocationAttemptJournalManifestV1,
    pub journal_receipt: InvocationAttemptJournalReceiptCommitment,
    pub state: EffectOutcomeStateV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectOutcomeObservationManifestV1 {
    pub schema_version: u16,
    pub observation_id: EffectOutcomeObservationId,
    pub subject: EffectOutcomeObservationSubjectV1,
    pub expected_store: EffectOutcomeStoreDescriptorV1,
    pub expected_head: EffectOutcomeHistoryHeadV1,
    pub observation_time_receipt: CurrentTimeReceiptCommitment,
    pub observation_latest_possible_unix_ms: u64,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectOutcomeStoreDispositionV1 {
    Appended {
        record: EffectOutcomeRecordCommitment,
        post_head: EffectOutcomeHistoryHeadV1,
    },
    ProvenNotAppended {
        observed_head: EffectOutcomeHistoryHeadV1,
        absence_evidence: EffectOutcomeAbsenceCommitment,
    },
    ObservationIdConflict {
        existing_manifest: EffectOutcomeObservationManifestCommitment,
    },
    HistoryHeadConflict {
        current_head: EffectOutcomeHistoryHeadV1,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectOutcomeStoreReceiptV1 {
    pub schema_version: u16,
    pub store: EffectOutcomeStoreDescriptorV1,
    pub manifest: EffectOutcomeObservationManifestV1,
    pub disposition: EffectOutcomeStoreDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: EffectOutcomeStoreReceiptCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectOutcomeHistoryHeadReceiptV1 {
    pub schema_version: u16,
    pub store: EffectOutcomeStoreDescriptorV1,
    pub journal_record: InvocationAttemptJournalRecordCommitment,
    pub head: EffectOutcomeHistoryHeadV1,
    pub latest_manifest: Option<EffectOutcomeObservationManifestV1>,
    pub valid_until: u64,
    pub receipt_commitment: EffectOutcomeHistoryHeadReceiptCommitment,
}

pub trait EffectOutcomeHistoryStoreV1 {
    type Error;

    fn descriptor(&self) -> EffectOutcomeStoreDescriptorV1;

    fn append_observation(
        &self,
        manifest: &EffectOutcomeObservationManifestV1,
    ) -> Result<EffectOutcomeStoreReceiptV1, Self::Error>;

    fn reconcile_observation(
        &self,
        manifest: &EffectOutcomeObservationManifestV1,
    ) -> Result<EffectOutcomeStoreReceiptV1, Self::Error>;

    fn read_attempt_head(
        &self,
        journal_record: InvocationAttemptJournalRecordCommitment,
    ) -> Result<EffectOutcomeHistoryHeadReceiptV1, Self::Error>;
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactDurablyJournaledActuatorEffectAttemptV1: sealed::Sealed {
    fn exact_journal_record(&self) -> InvocationAttemptJournalRecordCommitment;
    fn exact_journal_manifest(&self) -> InvocationAttemptJournalManifestV1;
    fn exact_journal_receipt_commitment(&self) -> InvocationAttemptJournalReceiptCommitment;
}

impl<S, P> sealed::Sealed for DurablyJournaledActuatorEffectAttemptV1<S, P> {}

impl<S, P> ExactDurablyJournaledActuatorEffectAttemptV1
    for DurablyJournaledActuatorEffectAttemptV1<S, P>
{
    fn exact_journal_record(&self) -> InvocationAttemptJournalRecordCommitment {
        self.record()
    }
    fn exact_journal_manifest(&self) -> InvocationAttemptJournalManifestV1 {
        self.manifest()
    }
    fn exact_journal_receipt_commitment(&self) -> InvocationAttemptJournalReceiptCommitment {
        self.receipt().receipt_commitment
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectOutcomeObservationPreparationError {
    UnsupportedStoreTimeBasis,
    StoreAlreadyExpired,
    ObservationTimeAlreadyExpired,
    JournalRecordMismatch,
    OutcomeSubjectMismatch,
    OutcomeAttemptMismatch,
    OutcomeRecoveryPolicyMismatch,
    TerminalOutcomeMissingActuatorReceipt,
    HistoryAlreadyTerminal,
}

pub struct EffectOutcomeObservationPreparationFailureV1<J, TQ> {
    journaled_attempt: J,
    outcome: ValidatedActuatorEffectOutcomeV1,
    observation_time: QualifiedCurrentTimeV1<TQ>,
    error: EffectOutcomeObservationPreparationError,
}

impl<J, TQ> EffectOutcomeObservationPreparationFailureV1<J, TQ> {
    pub const fn error(&self) -> EffectOutcomeObservationPreparationError {
        self.error
    }
    pub fn into_parts(
        self,
    ) -> (
        J,
        ValidatedActuatorEffectOutcomeV1,
        QualifiedCurrentTimeV1<TQ>,
    ) {
        (self.journaled_attempt, self.outcome, self.observation_time)
    }
}

pub struct PreparedEffectOutcomeObservationV1<J, TQ> {
    journaled_attempt: J,
    outcome: ValidatedActuatorEffectOutcomeV1,
    observation_time: QualifiedCurrentTimeV1<TQ>,
    expected_store: ExpectedEffectOutcomeStoreProfileV1,
    manifest: EffectOutcomeObservationManifestV1,
}

impl<J, TQ> PreparedEffectOutcomeObservationV1<J, TQ> {
    pub fn journaled_attempt(&self) -> &J {
        &self.journaled_attempt
    }
    pub const fn outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.outcome
    }
    pub const fn observation_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.observation_time
    }
    pub const fn manifest(&self) -> EffectOutcomeObservationManifestV1 {
        self.manifest
    }
}

fn state_from_validated_outcome(
    outcome: ValidatedActuatorEffectOutcomeV1,
) -> Result<EffectOutcomeStateV1, EffectOutcomeObservationPreparationError> {
    if outcome.recovery_policy() != recovery_policy_for(outcome.subject().recovery_mode) {
        return Err(EffectOutcomeObservationPreparationError::OutcomeRecoveryPolicyMismatch);
    }
    let receipt_commitment = outcome.receipt().map(|receipt| receipt.receipt_commitment);
    match outcome.disposition() {
        ValidatedActuatorEffectDispositionV1::Confirmed { outcome } => {
            let actuator_receipt = receipt_commitment.ok_or(
                EffectOutcomeObservationPreparationError::TerminalOutcomeMissingActuatorReceipt,
            )?;
            Ok(EffectOutcomeStateV1::Confirmed {
                outcome,
                actuator_receipt,
            })
        }
        ValidatedActuatorEffectDispositionV1::ProvenNotApplied { evidence } => {
            let actuator_receipt = receipt_commitment.ok_or(
                EffectOutcomeObservationPreparationError::TerminalOutcomeMissingActuatorReceipt,
            )?;
            Ok(EffectOutcomeStateV1::ProvenNotApplied {
                evidence,
                actuator_receipt,
            })
        }
        ValidatedActuatorEffectDispositionV1::OutcomeUnknown { reason } => {
            Ok(EffectOutcomeStateV1::OutcomeUnknown {
                reason,
                actuator_receipt: receipt_commitment,
                recovery_policy: outcome.recovery_policy(),
            })
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn prepare_effect_outcome_observation<J, TQ>(
    journaled_attempt: J,
    outcome: ValidatedActuatorEffectOutcomeV1,
    observation_time: QualifiedCurrentTimeV1<TQ>,
    observation_id: EffectOutcomeObservationId,
    expected_store: ExpectedEffectOutcomeStoreProfileV1,
    expected_head: ExpectedEffectOutcomeHistoryHeadV1,
) -> Result<
    PreparedEffectOutcomeObservationV1<J, TQ>,
    EffectOutcomeObservationPreparationFailureV1<J, TQ>,
>
where
    J: ExactDurablyJournaledActuatorEffectAttemptV1,
{
    let journal_record = journaled_attempt.exact_journal_record();
    let journal_manifest = journaled_attempt.exact_journal_manifest();
    let latest = observation_time.latest_possible_unix_ms();

    if expected_store.descriptor.time_basis != EffectOutcomeStoreTimeBasisV1::UnixMillisecondsUtc {
        return Err(EffectOutcomeObservationPreparationFailureV1 {
            journaled_attempt,
            outcome,
            observation_time,
            error: EffectOutcomeObservationPreparationError::UnsupportedStoreTimeBasis,
        });
    }
    if expected_store.descriptor.valid_until < latest {
        return Err(EffectOutcomeObservationPreparationFailureV1 {
            journaled_attempt,
            outcome,
            observation_time,
            error: EffectOutcomeObservationPreparationError::StoreAlreadyExpired,
        });
    }
    if observation_time.valid_until() < latest {
        return Err(EffectOutcomeObservationPreparationFailureV1 {
            journaled_attempt,
            outcome,
            observation_time,
            error: EffectOutcomeObservationPreparationError::ObservationTimeAlreadyExpired,
        });
    }
    if expected_head.head.journal_record() != journal_record {
        return Err(EffectOutcomeObservationPreparationFailureV1 {
            journaled_attempt,
            outcome,
            observation_time,
            error: EffectOutcomeObservationPreparationError::JournalRecordMismatch,
        });
    }
    if expected_head.head.terminal().is_some() {
        return Err(EffectOutcomeObservationPreparationFailureV1 {
            journaled_attempt,
            outcome,
            observation_time,
            error: EffectOutcomeObservationPreparationError::HistoryAlreadyTerminal,
        });
    }
    if outcome.subject() != journal_manifest.attempt.subject {
        return Err(EffectOutcomeObservationPreparationFailureV1 {
            journaled_attempt,
            outcome,
            observation_time,
            error: EffectOutcomeObservationPreparationError::OutcomeSubjectMismatch,
        });
    }
    if outcome.attempt_id() != journal_manifest.attempt.attempt_id {
        return Err(EffectOutcomeObservationPreparationFailureV1 {
            journaled_attempt,
            outcome,
            observation_time,
            error: EffectOutcomeObservationPreparationError::OutcomeAttemptMismatch,
        });
    }

    let state = match state_from_validated_outcome(outcome) {
        Ok(state) => state,
        Err(error) => {
            return Err(EffectOutcomeObservationPreparationFailureV1 {
                journaled_attempt,
                outcome,
                observation_time,
                error,
            });
        }
    };

    let manifest = EffectOutcomeObservationManifestV1 {
        schema_version: SSF_SCHEMA_V1,
        observation_id,
        subject: EffectOutcomeObservationSubjectV1 {
            journal_record,
            journal_manifest,
            journal_receipt: journaled_attempt.exact_journal_receipt_commitment(),
            state,
        },
        expected_store: expected_store.descriptor,
        expected_head: expected_head.head,
        observation_time_receipt: observation_time.receipt_commitment(),
        observation_latest_possible_unix_ms: latest,
        valid_until: expected_store
            .descriptor
            .valid_until
            .min(observation_time.valid_until()),
    };

    Ok(PreparedEffectOutcomeObservationV1 {
        journaled_attempt,
        outcome,
        observation_time,
        expected_store,
        manifest,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectOutcomeRecordingAmbiguityReasonV1 {
    StoreErrorAfterAppendBoundary,
    StoreDescriptorChangedAfter,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    ReceiptOutlivesObservation,
    ReceiptAlreadyExpiredAtObservation,
    OutcomeGenerationOverflow,
    InvalidPostHead,
    InvalidProvenNotAppendedHead,
    ObservationIdConflict,
    HistoryHeadConflict,
    StoreReportedOutcomeUnknown,
}

pub struct DurablyRecordedEffectOutcomeObservationV1<S, J, TQ> {
    journaled_attempt: J,
    outcome: ValidatedActuatorEffectOutcomeV1,
    observation_time: QualifiedCurrentTimeV1<TQ>,
    manifest: EffectOutcomeObservationManifestV1,
    receipt: EffectOutcomeStoreReceiptV1,
    record: EffectOutcomeRecordCommitment,
    post_head: EffectOutcomeHistoryHeadV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, J, TQ> DurablyRecordedEffectOutcomeObservationV1<S, J, TQ> {
    pub fn journaled_attempt(&self) -> &J {
        &self.journaled_attempt
    }
    pub const fn observation_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.observation_time
    }
    pub const fn actuator_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.outcome
    }
    pub const fn manifest(&self) -> EffectOutcomeObservationManifestV1 {
        self.manifest
    }
    pub const fn receipt(&self) -> EffectOutcomeStoreReceiptV1 {
        self.receipt
    }
    pub const fn record(&self) -> EffectOutcomeRecordCommitment {
        self.record
    }
    pub const fn post_head(&self) -> EffectOutcomeHistoryHeadV1 {
        self.post_head
    }
    pub const fn replay_authority_created(&self) -> bool {
        false
    }
}

pub struct RecoverableUnrecordedEffectOutcomeObservationV1<J, TQ> {
    journaled_attempt: J,
    outcome: ValidatedActuatorEffectOutcomeV1,
    observation_time: QualifiedCurrentTimeV1<TQ>,
    manifest: EffectOutcomeObservationManifestV1,
    receipt: EffectOutcomeStoreReceiptV1,
}

impl<J, TQ> RecoverableUnrecordedEffectOutcomeObservationV1<J, TQ> {
    pub const fn actuator_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.outcome
    }
    pub fn into_parts(
        self,
    ) -> (
        J,
        ValidatedActuatorEffectOutcomeV1,
        QualifiedCurrentTimeV1<TQ>,
    ) {
        (self.journaled_attempt, self.outcome, self.observation_time)
    }
    pub const fn manifest(&self) -> EffectOutcomeObservationManifestV1 {
        self.manifest
    }
    pub const fn receipt(&self) -> EffectOutcomeStoreReceiptV1 {
        self.receipt
    }
}

pub struct FrozenEffectOutcomeRecordingV1<S, J, TQ> {
    journaled_attempt: J,
    outcome: ValidatedActuatorEffectOutcomeV1,
    observation_time: QualifiedCurrentTimeV1<TQ>,
    manifest: EffectOutcomeObservationManifestV1,
    reason: EffectOutcomeRecordingAmbiguityReasonV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, J, TQ> FrozenEffectOutcomeRecordingV1<S, J, TQ> {
    pub fn journaled_attempt(&self) -> &J {
        &self.journaled_attempt
    }
    pub const fn observation_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.observation_time
    }
    pub const fn actuator_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.outcome
    }
    pub const fn reason(&self) -> EffectOutcomeRecordingAmbiguityReasonV1 {
        self.reason
    }
    pub const fn manifest(&self) -> EffectOutcomeObservationManifestV1 {
        self.manifest
    }
    pub const fn effect_disposition_rewritten_by_recording_failure(&self) -> bool {
        false
    }
    pub const fn replay_authority_created(&self) -> bool {
        false
    }
}

pub enum EffectOutcomeRecordingResultV1<S, J, TQ> {
    Recorded(DurablyRecordedEffectOutcomeObservationV1<S, J, TQ>),
    ProvenNotRecorded(RecoverableUnrecordedEffectOutcomeObservationV1<J, TQ>),
    RecordingOutcomeUnknown(FrozenEffectOutcomeRecordingV1<S, J, TQ>),
}

fn validate_post_head(
    manifest: EffectOutcomeObservationManifestV1,
    record: EffectOutcomeRecordCommitment,
    post: EffectOutcomeHistoryHeadV1,
) -> Result<(), EffectOutcomeRecordingAmbiguityReasonV1> {
    let expected_generation = manifest
        .expected_head
        .generation()
        .get()
        .checked_add(1)
        .ok_or(EffectOutcomeRecordingAmbiguityReasonV1::OutcomeGenerationOverflow)?;
    if post.journal_record() != manifest.subject.journal_record
        || post.generation().get() != expected_generation
        || post.head() != Some(record)
        || post.terminal() != manifest.subject.state.terminal()
    {
        return Err(EffectOutcomeRecordingAmbiguityReasonV1::InvalidPostHead);
    }
    Ok(())
}

fn absence_head_is_compatible(
    expected: EffectOutcomeHistoryHeadV1,
    observed: EffectOutcomeHistoryHeadV1,
) -> bool {
    if observed.journal_record() != expected.journal_record() || observed.generation() < expected.generation() {
        return false;
    }
    if observed.generation() == expected.generation() {
        return observed.head() == expected.head() && observed.terminal() == expected.terminal();
    }
    true
}

fn freeze<S, J, TQ>(
    prepared: PreparedEffectOutcomeObservationV1<J, TQ>,
    reason: EffectOutcomeRecordingAmbiguityReasonV1,
) -> EffectOutcomeRecordingResultV1<S, J, TQ> {
    EffectOutcomeRecordingResultV1::RecordingOutcomeUnknown(FrozenEffectOutcomeRecordingV1 {
        journaled_attempt: prepared.journaled_attempt,
        outcome: prepared.outcome,
        observation_time: prepared.observation_time,
        manifest: prepared.manifest,
        reason,
        _store: PhantomData,
    })
}

fn interpret_store_receipt<S, J, TQ>(
    prepared: PreparedEffectOutcomeObservationV1<J, TQ>,
    receipt: EffectOutcomeStoreReceiptV1,
) -> EffectOutcomeRecordingResultV1<S, J, TQ> {
    let manifest = prepared.manifest;
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::UnsupportedReceiptSchema);
    }
    if receipt.store != prepared.expected_store.descriptor {
        return freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::ReceiptStoreMismatch);
    }
    if receipt.manifest != manifest {
        return freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::ReceiptManifestMismatch);
    }
    if receipt.valid_until > prepared.expected_store.descriptor.valid_until {
        return freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::ReceiptOutlivesStore);
    }
    if receipt.valid_until > manifest.valid_until {
        return freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::ReceiptOutlivesObservation);
    }
    if receipt.valid_until < manifest.observation_latest_possible_unix_ms {
        return freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::ReceiptAlreadyExpiredAtObservation);
    }

    match receipt.disposition {
        EffectOutcomeStoreDispositionV1::Appended { record, post_head } => {
            if let Err(reason) = validate_post_head(manifest, record, post_head) {
                return freeze::<S, J, TQ>(prepared, reason);
            }
            EffectOutcomeRecordingResultV1::Recorded(DurablyRecordedEffectOutcomeObservationV1 {
                journaled_attempt: prepared.journaled_attempt,
                outcome: prepared.outcome,
                observation_time: prepared.observation_time,
                manifest,
                receipt,
                record,
                post_head,
                _store: PhantomData,
            })
        }
        EffectOutcomeStoreDispositionV1::ProvenNotAppended { observed_head, .. } => {
            if !absence_head_is_compatible(manifest.expected_head, observed_head) {
                return freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::InvalidProvenNotAppendedHead);
            }
            EffectOutcomeRecordingResultV1::ProvenNotRecorded(RecoverableUnrecordedEffectOutcomeObservationV1 {
                journaled_attempt: prepared.journaled_attempt,
                outcome: prepared.outcome,
                observation_time: prepared.observation_time,
                manifest,
                receipt,
            })
        }
        EffectOutcomeStoreDispositionV1::ObservationIdConflict { .. } => {
            freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::ObservationIdConflict)
        }
        EffectOutcomeStoreDispositionV1::HistoryHeadConflict { .. } => {
            freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::HistoryHeadConflict)
        }
        EffectOutcomeStoreDispositionV1::OutcomeUnknown => {
            freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::StoreReportedOutcomeUnknown)
        }
    }
}

pub fn append_effect_outcome_observation<S, J, TQ>(
    prepared: PreparedEffectOutcomeObservationV1<J, TQ>,
    store: &S,
) -> Result<EffectOutcomeRecordingResultV1<S, J, TQ>, PreparedEffectOutcomeObservationV1<J, TQ>>
where
    S: EffectOutcomeHistoryStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor {
        return Err(prepared);
    }
    let receipt = match store.append_observation(&prepared.manifest) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Ok(freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::StoreErrorAfterAppendBoundary));
        }
    };
    if store.descriptor() != prepared.expected_store.descriptor {
        return Ok(freeze::<S, J, TQ>(prepared, EffectOutcomeRecordingAmbiguityReasonV1::StoreDescriptorChangedAfter));
    }
    Ok(interpret_store_receipt::<S, J, TQ>(prepared, receipt))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectOutcomeHistoryReadError {
    StoreDescriptorMismatchBefore,
    StoreError,
    StoreDescriptorChangedAfter,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptJournalMismatch,
    HeadJournalMismatch,
    ReceiptOutlivesStore,
    LatestManifestJournalMismatch,
}

pub struct QualifiedEffectOutcomeHistoryHeadV1<S> {
    receipt: EffectOutcomeHistoryHeadReceiptV1,
    _store: PhantomData<fn() -> S>,
}

impl<S> QualifiedEffectOutcomeHistoryHeadV1<S> {
    pub const fn receipt(&self) -> EffectOutcomeHistoryHeadReceiptV1 {
        self.receipt
    }
    pub const fn head(&self) -> EffectOutcomeHistoryHeadV1 {
        self.receipt.head
    }
    pub const fn replay_authority_created(&self) -> bool {
        false
    }
}

pub fn read_effect_outcome_history_head<S>(
    store: &S,
    expected_store: ExpectedEffectOutcomeStoreProfileV1,
    journal_record: InvocationAttemptJournalRecordCommitment,
) -> Result<QualifiedEffectOutcomeHistoryHeadV1<S>, EffectOutcomeHistoryReadError>
where
    S: EffectOutcomeHistoryStoreV1,
{
    if store.descriptor() != expected_store.descriptor {
        return Err(EffectOutcomeHistoryReadError::StoreDescriptorMismatchBefore);
    }
    let receipt = store
        .read_attempt_head(journal_record)
        .map_err(|_| EffectOutcomeHistoryReadError::StoreError)?;
    if store.descriptor() != expected_store.descriptor {
        return Err(EffectOutcomeHistoryReadError::StoreDescriptorChangedAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(EffectOutcomeHistoryReadError::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_store.descriptor {
        return Err(EffectOutcomeHistoryReadError::ReceiptStoreMismatch);
    }
    if receipt.journal_record != journal_record {
        return Err(EffectOutcomeHistoryReadError::ReceiptJournalMismatch);
    }
    if receipt.head.journal_record() != journal_record {
        return Err(EffectOutcomeHistoryReadError::HeadJournalMismatch);
    }
    if receipt.valid_until > expected_store.descriptor.valid_until {
        return Err(EffectOutcomeHistoryReadError::ReceiptOutlivesStore);
    }
    if let Some(manifest) = receipt.latest_manifest {
        if manifest.subject.journal_record != journal_record {
            return Err(EffectOutcomeHistoryReadError::LatestManifestJournalMismatch);
        }
    }
    Ok(QualifiedEffectOutcomeHistoryHeadV1 {
        receipt,
        _store: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }
    fn journal_record(byte: u8) -> InvocationAttemptJournalRecordCommitment {
        InvocationAttemptJournalRecordCommitment::from_bytes(bytes(byte))
    }
    fn outcome_record(byte: u8) -> EffectOutcomeRecordCommitment {
        EffectOutcomeRecordCommitment::from_bytes(bytes(byte))
    }

    #[test]
    fn genesis_head_has_no_record_and_is_not_terminal() {
        let head = EffectOutcomeHistoryHeadV1::from_trusted_state(
            journal_record(1),
            EffectOutcomeHistoryGeneration::new(0),
            None,
            None,
        )
        .expect("valid genesis");
        assert!(head.permits_append());
    }

    #[test]
    fn genesis_cannot_be_terminal() {
        assert_eq!(
            EffectOutcomeHistoryHeadV1::from_trusted_state(
                journal_record(1),
                EffectOutcomeHistoryGeneration::new(0),
                None,
                Some(EffectOutcomeTerminalV1::Confirmed),
            ),
            Err(EffectOutcomeHistoryHeadError::GenesisIsTerminal)
        );
    }

    #[test]
    fn non_genesis_requires_a_record() {
        assert_eq!(
            EffectOutcomeHistoryHeadV1::from_trusted_state(
                journal_record(1),
                EffectOutcomeHistoryGeneration::new(1),
                None,
                None,
            ),
            Err(EffectOutcomeHistoryHeadError::NonGenesisMissingRecord)
        );
    }

    #[test]
    fn unknown_head_remains_refinable() {
        let head = EffectOutcomeHistoryHeadV1::from_trusted_state(
            journal_record(1),
            EffectOutcomeHistoryGeneration::new(1),
            Some(outcome_record(2)),
            None,
        )
        .expect("unknown head");
        assert!(head.permits_append());
    }

    #[test]
    fn terminal_heads_do_not_permit_append() {
        for terminal in [
            EffectOutcomeTerminalV1::Confirmed,
            EffectOutcomeTerminalV1::ProvenNotApplied,
        ] {
            let head = EffectOutcomeHistoryHeadV1::from_trusted_state(
                journal_record(1),
                EffectOutcomeHistoryGeneration::new(2),
                Some(outcome_record(3)),
                Some(terminal),
            )
            .expect("terminal head");
            assert!(!head.permits_append());
        }
    }

    #[test]
    fn durable_unknown_never_creates_replay_authority() {
        let state = EffectOutcomeStateV1::OutcomeUnknown {
            reason: ActuatorEffectAmbiguityReasonV1::ActuatorErrorAfterInvocation,
            actuator_receipt: None,
            recovery_policy: EffectRecoveryPolicyV1::NeverAutomaticRetry,
        };
        assert!(!state.replay_authority_created());
    }
}

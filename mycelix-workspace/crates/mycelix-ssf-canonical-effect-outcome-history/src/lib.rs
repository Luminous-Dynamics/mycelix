// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Append-only durable history for canonical SSF actuator outcome evidence.
//!
//! This layer persists the richer canonical evidence envelope produced after
//! execution. Historical evidence may outlive the authority that produced it;
//! only the recording/read operation itself must be freshly qualified.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_invocation_activation::InvocationJournalEvidenceV1;
use mycelix_ssf_canonical_actuator_execution::{
    CanonicalInvocationJournalEvidenceV1, CanonicalQualifiedEffectDispositionV1,
};
use mycelix_ssf_canonical_effect_outcome_evidence::CanonicalEffectOutcomeEvidenceV1;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_durable_actuator_invocation_attempt::{
    InvocationAttemptJournalDispositionV1, InvocationAttemptJournalRecordCommitment,
};
use mycelix_ssf_historical_actuator_invocation_journal::HistoricalInvocationJournalEvidenceV1;
use mycelix_ssf_replay_aware_invocation_journal::{
    ReplayInvocationJournalDispositionV1, ReplayInvocationJournalRecordCommitment,
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

digest_type!(CanonicalOutcomeObservationId);
digest_type!(CanonicalOutcomeRecordCommitment);
digest_type!(CanonicalOutcomeStoreReceiptCommitment);
digest_type!(CanonicalOutcomeAbsenceCommitment);
digest_type!(CanonicalOutcomeObservationManifestCommitment);
digest_type!(CanonicalOutcomeStoreIdentityCommitment);
digest_type!(CanonicalOutcomeStorePolicyCommitment);
digest_type!(CanonicalOutcomeHistoryHeadReceiptCommitment);

generation_type!(CanonicalOutcomeStoreGeneration);
generation_type!(CanonicalOutcomeHistoryGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalInvocationRecordV1 {
    Initial(InvocationAttemptJournalRecordCommitment),
    Replay(ReplayInvocationJournalRecordCommitment),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalOutcomeStoreTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CanonicalOutcomeStoreDescriptorV1 {
    pub stable_identity: CanonicalOutcomeStoreIdentityCommitment,
    pub policy: CanonicalOutcomeStorePolicyCommitment,
    pub generation: CanonicalOutcomeStoreGeneration,
    pub time_basis: CanonicalOutcomeStoreTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCanonicalOutcomeStoreProfileV1 {
    descriptor: CanonicalOutcomeStoreDescriptorV1,
}

impl ExpectedCanonicalOutcomeStoreProfileV1 {
    pub const fn from_trusted_configuration(descriptor: CanonicalOutcomeStoreDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> CanonicalOutcomeStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalOutcomeTerminalV1 {
    Confirmed,
    ProvenNotApplied,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalOutcomeHistoryHeadErrorV1 {
    GenesisHasRecord,
    GenesisIsTerminal,
    NonGenesisMissingRecord,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CanonicalOutcomeHistoryHeadV1 {
    invocation_record: CanonicalInvocationRecordV1,
    generation: CanonicalOutcomeHistoryGeneration,
    head: Option<CanonicalOutcomeRecordCommitment>,
    terminal: Option<CanonicalOutcomeTerminalV1>,
}

impl CanonicalOutcomeHistoryHeadV1 {
    pub fn from_trusted_state(
        invocation_record: CanonicalInvocationRecordV1,
        generation: CanonicalOutcomeHistoryGeneration,
        head: Option<CanonicalOutcomeRecordCommitment>,
        terminal: Option<CanonicalOutcomeTerminalV1>,
    ) -> Result<Self, CanonicalOutcomeHistoryHeadErrorV1> {
        match (generation.get(), head, terminal) {
            (0, Some(_), _) => return Err(CanonicalOutcomeHistoryHeadErrorV1::GenesisHasRecord),
            (0, None, Some(_)) => return Err(CanonicalOutcomeHistoryHeadErrorV1::GenesisIsTerminal),
            (0, None, None) => {}
            (_, None, _) => return Err(CanonicalOutcomeHistoryHeadErrorV1::NonGenesisMissingRecord),
            (_, Some(_), _) => {}
        }
        Ok(Self {
            invocation_record,
            generation,
            head,
            terminal,
        })
    }

    pub const fn invocation_record(&self) -> CanonicalInvocationRecordV1 {
        self.invocation_record
    }

    pub const fn generation(&self) -> CanonicalOutcomeHistoryGeneration {
        self.generation
    }

    pub const fn head(&self) -> Option<CanonicalOutcomeRecordCommitment> {
        self.head
    }

    pub const fn terminal(&self) -> Option<CanonicalOutcomeTerminalV1> {
        self.terminal
    }

    pub const fn permits_append(&self) -> bool {
        self.terminal.is_none()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCanonicalOutcomeHistoryHeadV1 {
    head: CanonicalOutcomeHistoryHeadV1,
}

impl ExpectedCanonicalOutcomeHistoryHeadV1 {
    pub const fn from_trusted_state(head: CanonicalOutcomeHistoryHeadV1) -> Self {
        Self { head }
    }

    pub const fn head(&self) -> CanonicalOutcomeHistoryHeadV1 {
        self.head
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalOutcomeObservationSubjectV1 {
    pub invocation_record: CanonicalInvocationRecordV1,
    pub evidence: CanonicalEffectOutcomeEvidenceV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalOutcomeObservationManifestV1 {
    pub schema_version: u16,
    pub observation_id: CanonicalOutcomeObservationId,
    pub subject: CanonicalOutcomeObservationSubjectV1,
    pub expected_store: CanonicalOutcomeStoreDescriptorV1,
    pub expected_head: CanonicalOutcomeHistoryHeadV1,
    pub recording_time_receipt: CurrentTimeReceiptCommitment,
    pub recording_latest_possible_unix_ms: u64,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalOutcomeHistoryEntryV1 {
    pub record: CanonicalOutcomeRecordCommitment,
    pub manifest: CanonicalOutcomeObservationManifestV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalOutcomeStoreDispositionV1 {
    Appended {
        record: CanonicalOutcomeRecordCommitment,
        post_head: CanonicalOutcomeHistoryHeadV1,
    },
    ProvenNotAppended {
        observed_head: CanonicalOutcomeHistoryHeadV1,
        absence_evidence: CanonicalOutcomeAbsenceCommitment,
    },
    ObservationIdConflict {
        existing_manifest: CanonicalOutcomeObservationManifestCommitment,
    },
    HistoryHeadConflict {
        current_head: CanonicalOutcomeHistoryHeadV1,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalOutcomeStoreReceiptV1 {
    pub schema_version: u16,
    pub store: CanonicalOutcomeStoreDescriptorV1,
    pub manifest: CanonicalOutcomeObservationManifestV1,
    pub disposition: CanonicalOutcomeStoreDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: CanonicalOutcomeStoreReceiptCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalOutcomeHistoryHeadReceiptV1 {
    pub schema_version: u16,
    pub store: CanonicalOutcomeStoreDescriptorV1,
    pub invocation_record: CanonicalInvocationRecordV1,
    pub head: CanonicalOutcomeHistoryHeadV1,
    pub latest_entry: Option<CanonicalOutcomeHistoryEntryV1>,
    pub valid_until: u64,
    pub receipt_commitment: CanonicalOutcomeHistoryHeadReceiptCommitment,
}

pub trait CanonicalOutcomeHistoryStoreV1 {
    type Error;

    fn descriptor(&self) -> CanonicalOutcomeStoreDescriptorV1;

    fn append_observation(
        &self,
        manifest: &CanonicalOutcomeObservationManifestV1,
    ) -> Result<CanonicalOutcomeStoreReceiptV1, Self::Error>;

    fn reconcile_observation(
        &self,
        manifest: &CanonicalOutcomeObservationManifestV1,
    ) -> Result<CanonicalOutcomeStoreReceiptV1, Self::Error>;

    fn read_invocation_head(
        &self,
        invocation_record: CanonicalInvocationRecordV1,
    ) -> Result<CanonicalOutcomeHistoryHeadReceiptV1, Self::Error>;
}

fn invocation_record_from_evidence(
    evidence: CanonicalEffectOutcomeEvidenceV1,
) -> Option<CanonicalInvocationRecordV1> {
    match evidence.journal_audit().evidence {
        CanonicalInvocationJournalEvidenceV1::CurrentOrSameGeneration(
            InvocationJournalEvidenceV1::Initial { record, .. },
        ) => Some(CanonicalInvocationRecordV1::Initial(record)),
        CanonicalInvocationJournalEvidenceV1::CurrentOrSameGeneration(
            InvocationJournalEvidenceV1::Replay { record, .. },
        ) => Some(CanonicalInvocationRecordV1::Replay(record)),
        CanonicalInvocationJournalEvidenceV1::HistoricalSameIdentity(
            HistoricalInvocationJournalEvidenceV1::Initial(receipt),
        ) => match receipt.disposition {
            InvocationAttemptJournalDispositionV1::Journaled { record, .. } => {
                Some(CanonicalInvocationRecordV1::Initial(record))
            }
            _ => None,
        },
        CanonicalInvocationJournalEvidenceV1::HistoricalSameIdentity(
            HistoricalInvocationJournalEvidenceV1::Replay(receipt),
        ) => match receipt.disposition {
            ReplayInvocationJournalDispositionV1::Journaled { record, .. } => {
                Some(CanonicalInvocationRecordV1::Replay(record))
            }
            _ => None,
        },
    }
}

fn canonical_terminal(
    evidence: CanonicalEffectOutcomeEvidenceV1,
) -> Option<CanonicalOutcomeTerminalV1> {
    match evidence.canonical_disposition() {
        CanonicalQualifiedEffectDispositionV1::Confirmed { .. } => {
            Some(CanonicalOutcomeTerminalV1::Confirmed)
        }
        CanonicalQualifiedEffectDispositionV1::ProvenNotApplied { .. } => {
            Some(CanonicalOutcomeTerminalV1::ProvenNotApplied)
        }
        CanonicalQualifiedEffectDispositionV1::OutcomeUnknown { .. } => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalOutcomePreparationErrorV1 {
    UnsupportedStoreTimeBasis,
    StoreAlreadyExpired,
    RecordingTimeAlreadyExpired,
    RecordingTimeRegressedBeforePreInvocation,
    RecordingTimeRegressedBeforePostInvocation,
    JournalEvidenceNotDurablyJournaled,
    HistoryInvocationMismatch,
    HistoryAlreadyTerminal,
}

pub struct CanonicalOutcomePreparationFailureV1<TQ> {
    evidence: CanonicalEffectOutcomeEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    error: CanonicalOutcomePreparationErrorV1,
}

impl<TQ> CanonicalOutcomePreparationFailureV1<TQ> {
    pub const fn error(&self) -> CanonicalOutcomePreparationErrorV1 {
        self.error
    }

    pub fn into_parts(self) -> (CanonicalEffectOutcomeEvidenceV1, QualifiedCurrentTimeV1<TQ>) {
        (self.evidence, self.recording_time)
    }
}

pub struct PreparedCanonicalOutcomeObservationV1<TQ> {
    evidence: CanonicalEffectOutcomeEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    expected_store: ExpectedCanonicalOutcomeStoreProfileV1,
    manifest: CanonicalOutcomeObservationManifestV1,
}

impl<TQ> PreparedCanonicalOutcomeObservationV1<TQ> {
    pub const fn evidence(&self) -> CanonicalEffectOutcomeEvidenceV1 {
        self.evidence
    }

    pub const fn recording_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.recording_time
    }

    pub const fn manifest(&self) -> CanonicalOutcomeObservationManifestV1 {
        self.manifest
    }
}

#[allow(clippy::too_many_arguments)]
pub fn prepare_canonical_outcome_observation<TQ>(
    evidence: CanonicalEffectOutcomeEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    observation_id: CanonicalOutcomeObservationId,
    expected_store: ExpectedCanonicalOutcomeStoreProfileV1,
    expected_head: ExpectedCanonicalOutcomeHistoryHeadV1,
) -> Result<PreparedCanonicalOutcomeObservationV1<TQ>, CanonicalOutcomePreparationFailureV1<TQ>> {
    let latest = recording_time.latest_possible_unix_ms();
    let invocation_record = invocation_record_from_evidence(evidence);

    let error = if expected_store.descriptor.time_basis
        != CanonicalOutcomeStoreTimeBasisV1::UnixMillisecondsUtc
    {
        Some(CanonicalOutcomePreparationErrorV1::UnsupportedStoreTimeBasis)
    } else if expected_store.descriptor.valid_until < latest {
        Some(CanonicalOutcomePreparationErrorV1::StoreAlreadyExpired)
    } else if recording_time.valid_until() < latest {
        Some(CanonicalOutcomePreparationErrorV1::RecordingTimeAlreadyExpired)
    } else if latest < evidence.pre_invocation_time().latest_possible_unix_ms {
        Some(CanonicalOutcomePreparationErrorV1::RecordingTimeRegressedBeforePreInvocation)
    } else if evidence
        .post_invocation_time()
        .is_some_and(|post| latest < post.latest_possible_unix_ms)
    {
        Some(CanonicalOutcomePreparationErrorV1::RecordingTimeRegressedBeforePostInvocation)
    } else if invocation_record.is_none() {
        Some(CanonicalOutcomePreparationErrorV1::JournalEvidenceNotDurablyJournaled)
    } else if expected_head.head.invocation_record() != invocation_record.unwrap() {
        Some(CanonicalOutcomePreparationErrorV1::HistoryInvocationMismatch)
    } else if !expected_head.head.permits_append() {
        Some(CanonicalOutcomePreparationErrorV1::HistoryAlreadyTerminal)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(CanonicalOutcomePreparationFailureV1 {
            evidence,
            recording_time,
            error,
        });
    }

    let manifest = CanonicalOutcomeObservationManifestV1 {
        schema_version: SSF_SCHEMA_V1,
        observation_id,
        subject: CanonicalOutcomeObservationSubjectV1 {
            invocation_record: invocation_record.expect("validated above"),
            evidence,
        },
        expected_store: expected_store.descriptor,
        expected_head: expected_head.head,
        recording_time_receipt: recording_time.receipt_commitment(),
        recording_latest_possible_unix_ms: latest,
        valid_until: expected_store
            .descriptor
            .valid_until
            .min(recording_time.valid_until()),
    };

    Ok(PreparedCanonicalOutcomeObservationV1 {
        evidence,
        recording_time,
        expected_store,
        manifest,
    })
}

fn next_head_matches(
    manifest: CanonicalOutcomeObservationManifestV1,
    record: CanonicalOutcomeRecordCommitment,
    post_head: CanonicalOutcomeHistoryHeadV1,
) -> bool {
    let Some(next) = manifest.expected_head.generation().get().checked_add(1) else {
        return false;
    };
    post_head.invocation_record() == manifest.subject.invocation_record
        && post_head.generation().get() == next
        && post_head.head() == Some(record)
        && post_head.terminal() == canonical_terminal(manifest.subject.evidence)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalOutcomeRecordingAmbiguityReasonV1 {
    StoreErrorAfterAppendBoundary,
    StoreDescriptorChangedAfter,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptManifestMismatch,
    ReceiptOutlivesStore,
    ReceiptOutlivesObservation,
    ReceiptAlreadyExpiredAtRecording,
    InvalidPostHead,
    InvalidProvenNotAppendedHead,
    ObservationIdConflict,
    HistoryHeadConflict,
    StoreReportedOutcomeUnknown,
}

pub struct DurablyRecordedCanonicalOutcomeV1<S, TQ> {
    evidence: CanonicalEffectOutcomeEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    manifest: CanonicalOutcomeObservationManifestV1,
    receipt: CanonicalOutcomeStoreReceiptV1,
    record: CanonicalOutcomeRecordCommitment,
    post_head: CanonicalOutcomeHistoryHeadV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, TQ> DurablyRecordedCanonicalOutcomeV1<S, TQ> {
    pub const fn evidence(&self) -> CanonicalEffectOutcomeEvidenceV1 {
        self.evidence
    }

    pub const fn manifest(&self) -> CanonicalOutcomeObservationManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> CanonicalOutcomeStoreReceiptV1 {
        self.receipt
    }

    pub const fn record(&self) -> CanonicalOutcomeRecordCommitment {
        self.record
    }

    pub const fn post_head(&self) -> CanonicalOutcomeHistoryHeadV1 {
        self.post_head
    }

    pub const fn recording_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.recording_time
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

pub struct RecoverableCanonicalOutcomeObservationV1<TQ> {
    evidence: CanonicalEffectOutcomeEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    manifest: CanonicalOutcomeObservationManifestV1,
    receipt: CanonicalOutcomeStoreReceiptV1,
}

impl<TQ> RecoverableCanonicalOutcomeObservationV1<TQ> {
    pub fn into_parts(self) -> (CanonicalEffectOutcomeEvidenceV1, QualifiedCurrentTimeV1<TQ>) {
        (self.evidence, self.recording_time)
    }

    pub const fn manifest(&self) -> CanonicalOutcomeObservationManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> CanonicalOutcomeStoreReceiptV1 {
        self.receipt
    }
}

pub struct FrozenCanonicalOutcomeRecordingV1<S, TQ> {
    evidence: CanonicalEffectOutcomeEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    manifest: CanonicalOutcomeObservationManifestV1,
    reason: CanonicalOutcomeRecordingAmbiguityReasonV1,
    receipt: Option<CanonicalOutcomeStoreReceiptV1>,
    _store: PhantomData<fn() -> S>,
}

impl<S, TQ> FrozenCanonicalOutcomeRecordingV1<S, TQ> {
    pub const fn evidence(&self) -> CanonicalEffectOutcomeEvidenceV1 {
        self.evidence
    }

    pub const fn reason(&self) -> CanonicalOutcomeRecordingAmbiguityReasonV1 {
        self.reason
    }

    pub const fn receipt(&self) -> Option<CanonicalOutcomeStoreReceiptV1> {
        self.receipt
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

pub enum CanonicalOutcomeRecordingResultV1<S, TQ> {
    Recorded(DurablyRecordedCanonicalOutcomeV1<S, TQ>),
    ProvenNotRecorded(RecoverableCanonicalOutcomeObservationV1<TQ>),
    RecordingOutcomeUnknown(FrozenCanonicalOutcomeRecordingV1<S, TQ>),
}

fn freeze_recording<S, TQ>(
    prepared: PreparedCanonicalOutcomeObservationV1<TQ>,
    reason: CanonicalOutcomeRecordingAmbiguityReasonV1,
    receipt: Option<CanonicalOutcomeStoreReceiptV1>,
) -> CanonicalOutcomeRecordingResultV1<S, TQ> {
    CanonicalOutcomeRecordingResultV1::RecordingOutcomeUnknown(FrozenCanonicalOutcomeRecordingV1 {
        evidence: prepared.evidence,
        recording_time: prepared.recording_time,
        manifest: prepared.manifest,
        reason,
        receipt,
        _store: PhantomData,
    })
}

fn interpret_store_result<S, TQ>(
    prepared: PreparedCanonicalOutcomeObservationV1<TQ>,
    descriptor_after: CanonicalOutcomeStoreDescriptorV1,
    result: Result<CanonicalOutcomeStoreReceiptV1, S::Error>,
) -> CanonicalOutcomeRecordingResultV1<S, TQ>
where
    S: CanonicalOutcomeHistoryStoreV1,
{
    let receipt = match result {
        Ok(receipt) => receipt,
        Err(_) => {
            return freeze_recording::<S, TQ>(
                prepared,
                CanonicalOutcomeRecordingAmbiguityReasonV1::StoreErrorAfterAppendBoundary,
                None,
            );
        }
    };

    if descriptor_after != prepared.expected_store.descriptor {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::StoreDescriptorChangedAfter,
            Some(receipt),
        );
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::UnsupportedReceiptSchema,
            Some(receipt),
        );
    }
    if receipt.store != prepared.expected_store.descriptor {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::ReceiptStoreMismatch,
            Some(receipt),
        );
    }
    if receipt.manifest != prepared.manifest {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::ReceiptManifestMismatch,
            Some(receipt),
        );
    }
    if receipt.valid_until > prepared.expected_store.descriptor.valid_until {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::ReceiptOutlivesStore,
            Some(receipt),
        );
    }
    if receipt.valid_until > prepared.manifest.valid_until {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::ReceiptOutlivesObservation,
            Some(receipt),
        );
    }
    if receipt.valid_until < prepared.manifest.recording_latest_possible_unix_ms {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::ReceiptAlreadyExpiredAtRecording,
            Some(receipt),
        );
    }

    match receipt.disposition {
        CanonicalOutcomeStoreDispositionV1::Appended { record, post_head } => {
            if !next_head_matches(prepared.manifest, record, post_head) {
                return freeze_recording::<S, TQ>(
                    prepared,
                    CanonicalOutcomeRecordingAmbiguityReasonV1::InvalidPostHead,
                    Some(receipt),
                );
            }
            CanonicalOutcomeRecordingResultV1::Recorded(DurablyRecordedCanonicalOutcomeV1 {
                evidence: prepared.evidence,
                recording_time: prepared.recording_time,
                manifest: prepared.manifest,
                receipt,
                record,
                post_head,
                _store: PhantomData,
            })
        }
        CanonicalOutcomeStoreDispositionV1::ProvenNotAppended { observed_head, .. } => {
            if observed_head != prepared.manifest.expected_head {
                return freeze_recording::<S, TQ>(
                    prepared,
                    CanonicalOutcomeRecordingAmbiguityReasonV1::InvalidProvenNotAppendedHead,
                    Some(receipt),
                );
            }
            CanonicalOutcomeRecordingResultV1::ProvenNotRecorded(
                RecoverableCanonicalOutcomeObservationV1 {
                    evidence: prepared.evidence,
                    recording_time: prepared.recording_time,
                    manifest: prepared.manifest,
                    receipt,
                },
            )
        }
        CanonicalOutcomeStoreDispositionV1::ObservationIdConflict { .. } => freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::ObservationIdConflict,
            Some(receipt),
        ),
        CanonicalOutcomeStoreDispositionV1::HistoryHeadConflict { .. } => freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::HistoryHeadConflict,
            Some(receipt),
        ),
        CanonicalOutcomeStoreDispositionV1::OutcomeUnknown => freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            Some(receipt),
        ),
    }
}

pub fn append_canonical_outcome<S, TQ>(
    prepared: PreparedCanonicalOutcomeObservationV1<TQ>,
    store: &S,
) -> CanonicalOutcomeRecordingResultV1<S, TQ>
where
    S: CanonicalOutcomeHistoryStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::StoreDescriptorChangedAfter,
            None,
        );
    }
    let result = store.append_observation(&prepared.manifest);
    let descriptor_after = store.descriptor();
    interpret_store_result::<S, TQ>(prepared, descriptor_after, result)
}

pub fn reconcile_canonical_outcome<S, TQ>(
    prepared: PreparedCanonicalOutcomeObservationV1<TQ>,
    store: &S,
) -> CanonicalOutcomeRecordingResultV1<S, TQ>
where
    S: CanonicalOutcomeHistoryStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalOutcomeRecordingAmbiguityReasonV1::StoreDescriptorChangedAfter,
            None,
        );
    }
    let result = store.reconcile_observation(&prepared.manifest);
    let descriptor_after = store.descriptor();
    interpret_store_result::<S, TQ>(prepared, descriptor_after, result)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalOutcomeHeadReadErrorV1 {
    UnsupportedStoreTimeBasis,
    StoreAlreadyExpired,
    ReadTimeAlreadyExpired,
    StoreDescriptorMismatchBefore,
    StoreDescriptorChangedAfter,
    ReadFailed,
    UnsupportedReceiptSchema,
    ReceiptStoreMismatch,
    ReceiptInvocationMismatch,
    ReceiptHeadInvocationMismatch,
    ReceiptOutlivesStore,
    ReceiptAlreadyExpired,
    GenesisHasLatestEntry,
    NonGenesisMissingLatestEntry,
    LatestEntryRecordMismatch,
    LatestEntryInvocationMismatch,
    LatestEntryGenerationMismatch,
    LatestEntryTerminalMismatch,
}

pub struct QualifiedCanonicalOutcomeHistoryHeadV1<S, TQ> {
    expected_store: ExpectedCanonicalOutcomeStoreProfileV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    receipt: CanonicalOutcomeHistoryHeadReceiptV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, TQ> QualifiedCanonicalOutcomeHistoryHeadV1<S, TQ> {
    pub const fn receipt(&self) -> CanonicalOutcomeHistoryHeadReceiptV1 {
        self.receipt
    }

    pub const fn head(&self) -> CanonicalOutcomeHistoryHeadV1 {
        self.receipt.head
    }

    pub const fn latest_entry(&self) -> Option<CanonicalOutcomeHistoryEntryV1> {
        self.receipt.latest_entry
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

pub fn read_canonical_outcome_history_head<S, TQ>(
    invocation_record: CanonicalInvocationRecordV1,
    expected_store: ExpectedCanonicalOutcomeStoreProfileV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    store: &S,
) -> Result<QualifiedCanonicalOutcomeHistoryHeadV1<S, TQ>, CanonicalOutcomeHeadReadErrorV1>
where
    S: CanonicalOutcomeHistoryStoreV1,
{
    let latest = current_time.latest_possible_unix_ms();
    if expected_store.descriptor.time_basis != CanonicalOutcomeStoreTimeBasisV1::UnixMillisecondsUtc {
        return Err(CanonicalOutcomeHeadReadErrorV1::UnsupportedStoreTimeBasis);
    }
    if expected_store.descriptor.valid_until < latest {
        return Err(CanonicalOutcomeHeadReadErrorV1::StoreAlreadyExpired);
    }
    if current_time.valid_until() < latest {
        return Err(CanonicalOutcomeHeadReadErrorV1::ReadTimeAlreadyExpired);
    }
    if store.descriptor() != expected_store.descriptor {
        return Err(CanonicalOutcomeHeadReadErrorV1::StoreDescriptorMismatchBefore);
    }

    let receipt = store
        .read_invocation_head(invocation_record)
        .map_err(|_| CanonicalOutcomeHeadReadErrorV1::ReadFailed)?;
    if store.descriptor() != expected_store.descriptor {
        return Err(CanonicalOutcomeHeadReadErrorV1::StoreDescriptorChangedAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(CanonicalOutcomeHeadReadErrorV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_store.descriptor {
        return Err(CanonicalOutcomeHeadReadErrorV1::ReceiptStoreMismatch);
    }
    if receipt.invocation_record != invocation_record {
        return Err(CanonicalOutcomeHeadReadErrorV1::ReceiptInvocationMismatch);
    }
    if receipt.head.invocation_record() != invocation_record {
        return Err(CanonicalOutcomeHeadReadErrorV1::ReceiptHeadInvocationMismatch);
    }
    if receipt.valid_until > expected_store.descriptor.valid_until {
        return Err(CanonicalOutcomeHeadReadErrorV1::ReceiptOutlivesStore);
    }
    if receipt.valid_until < latest {
        return Err(CanonicalOutcomeHeadReadErrorV1::ReceiptAlreadyExpired);
    }

    match (receipt.head.generation().get(), receipt.head.head(), receipt.latest_entry) {
        (0, None, None) => {}
        (0, None, Some(_)) => return Err(CanonicalOutcomeHeadReadErrorV1::GenesisHasLatestEntry),
        (_, Some(_), None) => {
            return Err(CanonicalOutcomeHeadReadErrorV1::NonGenesisMissingLatestEntry)
        }
        (_, Some(head_record), Some(entry)) => {
            if entry.record != head_record {
                return Err(CanonicalOutcomeHeadReadErrorV1::LatestEntryRecordMismatch);
            }
            if entry.manifest.subject.invocation_record != invocation_record {
                return Err(CanonicalOutcomeHeadReadErrorV1::LatestEntryInvocationMismatch);
            }
            let next = entry
                .manifest
                .expected_head
                .generation()
                .get()
                .checked_add(1)
                .ok_or(CanonicalOutcomeHeadReadErrorV1::LatestEntryGenerationMismatch)?;
            if next != receipt.head.generation().get()
                || entry.manifest.expected_head.invocation_record() != invocation_record
                || entry.manifest.expected_head.head()
                    == Some(entry.record)
            {
                return Err(CanonicalOutcomeHeadReadErrorV1::LatestEntryGenerationMismatch);
            }
            if canonical_terminal(entry.manifest.subject.evidence) != receipt.head.terminal() {
                return Err(CanonicalOutcomeHeadReadErrorV1::LatestEntryTerminalMismatch);
            }
        }
        (_, None, _) => return Err(CanonicalOutcomeHeadReadErrorV1::NonGenesisMissingLatestEntry),
    }

    Ok(QualifiedCanonicalOutcomeHistoryHeadV1 {
        expected_store,
        current_time,
        receipt,
        _store: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terminal_state_is_derived_only_from_canonical_disposition() {
        let unknown = CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
            low_level_disposition:
                mycelix_ssf_actuator_effect_protocol::ValidatedActuatorEffectDispositionV1::OutcomeUnknown {
                    reason: mycelix_ssf_actuator_effect_protocol::ActuatorEffectAmbiguityReasonV1::StoreReportedOutcomeUnknown,
                },
            qualified_actuator_receipt: None,
            post_invocation_ambiguity: None,
            recovery_policy: mycelix_ssf_actuator_effect_protocol::EffectRecoveryPolicyV1::NeverAutomaticRetry,
        };
        assert!(!unknown.is_terminal());
    }

    #[test]
    fn genesis_head_has_no_record() {
        let invocation = CanonicalInvocationRecordV1::Initial(
            InvocationAttemptJournalRecordCommitment::from_bytes([1; 32]),
        );
        let head = CanonicalOutcomeHistoryHeadV1::from_trusted_state(
            invocation,
            CanonicalOutcomeHistoryGeneration::new(0),
            None,
            None,
        )
        .unwrap();
        assert!(head.permits_append());
    }
}
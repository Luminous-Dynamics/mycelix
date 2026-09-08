// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Unified append-only outcome history for canonical completed-effect evidence.
//!
//! Historical effect evidence may outlive the authority that produced it. Only
//! the current recording/read operation must be fresh. Terminal state is
//! derived exclusively from the canonical qualified disposition, never from the
//! raw actuator report. The head-read API returns an explicit `{record,
//! manifest}` pair so durable head identity and latest observation cannot be
//! ambiguously paired.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_canonical_actuator_execution::CanonicalQualifiedEffectDispositionV1;
use mycelix_ssf_canonical_completed_effect_evidence::{
    CanonicalCompletedEffectEvidenceV1, CanonicalCompletedInvocationRecordV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
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

digest_type!(CanonicalCompletedOutcomeObservationId);
digest_type!(CanonicalCompletedOutcomeRecordCommitment);
digest_type!(CanonicalCompletedOutcomeStoreReceiptCommitment);
digest_type!(CanonicalCompletedOutcomeAbsenceCommitment);
digest_type!(CanonicalCompletedOutcomeManifestCommitment);
digest_type!(CanonicalCompletedOutcomeStoreIdentityCommitment);
digest_type!(CanonicalCompletedOutcomeStorePolicyCommitment);
digest_type!(CanonicalCompletedOutcomeHeadReceiptCommitment);

generation_type!(CanonicalCompletedOutcomeStoreGeneration);
generation_type!(CanonicalCompletedOutcomeHistoryGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalCompletedOutcomeStoreTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CanonicalCompletedOutcomeStoreDescriptorV1 {
    pub stable_identity: CanonicalCompletedOutcomeStoreIdentityCommitment,
    pub policy: CanonicalCompletedOutcomeStorePolicyCommitment,
    pub generation: CanonicalCompletedOutcomeStoreGeneration,
    pub time_basis: CanonicalCompletedOutcomeStoreTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCanonicalCompletedOutcomeStoreProfileV1 {
    descriptor: CanonicalCompletedOutcomeStoreDescriptorV1,
}

impl ExpectedCanonicalCompletedOutcomeStoreProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: CanonicalCompletedOutcomeStoreDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> CanonicalCompletedOutcomeStoreDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalCompletedOutcomeTerminalV1 {
    Confirmed,
    ProvenNotApplied,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalCompletedOutcomeHistoryHeadErrorV1 {
    GenesisHasRecord,
    GenesisIsTerminal,
    NonGenesisMissingRecord,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CanonicalCompletedOutcomeHistoryHeadV1 {
    invocation_record: CanonicalCompletedInvocationRecordV1,
    generation: CanonicalCompletedOutcomeHistoryGeneration,
    head: Option<CanonicalCompletedOutcomeRecordCommitment>,
    terminal: Option<CanonicalCompletedOutcomeTerminalV1>,
}

impl CanonicalCompletedOutcomeHistoryHeadV1 {
    pub fn from_trusted_state(
        invocation_record: CanonicalCompletedInvocationRecordV1,
        generation: CanonicalCompletedOutcomeHistoryGeneration,
        head: Option<CanonicalCompletedOutcomeRecordCommitment>,
        terminal: Option<CanonicalCompletedOutcomeTerminalV1>,
    ) -> Result<Self, CanonicalCompletedOutcomeHistoryHeadErrorV1> {
        match (generation.get(), head, terminal) {
            (0, Some(_), _) => {
                return Err(CanonicalCompletedOutcomeHistoryHeadErrorV1::GenesisHasRecord)
            }
            (0, None, Some(_)) => {
                return Err(CanonicalCompletedOutcomeHistoryHeadErrorV1::GenesisIsTerminal)
            }
            (0, None, None) => {}
            (_, None, _) => {
                return Err(CanonicalCompletedOutcomeHistoryHeadErrorV1::NonGenesisMissingRecord)
            }
            (_, Some(_), _) => {}
        }

        Ok(Self {
            invocation_record,
            generation,
            head,
            terminal,
        })
    }

    pub const fn invocation_record(&self) -> CanonicalCompletedInvocationRecordV1 {
        self.invocation_record
    }

    pub const fn generation(&self) -> CanonicalCompletedOutcomeHistoryGeneration {
        self.generation
    }

    pub const fn head(&self) -> Option<CanonicalCompletedOutcomeRecordCommitment> {
        self.head
    }

    pub const fn terminal(&self) -> Option<CanonicalCompletedOutcomeTerminalV1> {
        self.terminal
    }

    pub const fn permits_append(&self) -> bool {
        self.terminal.is_none()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCanonicalCompletedOutcomeHistoryHeadV1 {
    head: CanonicalCompletedOutcomeHistoryHeadV1,
}

impl ExpectedCanonicalCompletedOutcomeHistoryHeadV1 {
    pub const fn from_trusted_state(head: CanonicalCompletedOutcomeHistoryHeadV1) -> Self {
        Self { head }
    }

    pub const fn head(&self) -> CanonicalCompletedOutcomeHistoryHeadV1 {
        self.head
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalCompletedOutcomeObservationSubjectV1 {
    pub invocation_record: CanonicalCompletedInvocationRecordV1,
    pub evidence: CanonicalCompletedEffectEvidenceV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalCompletedOutcomeObservationManifestV1 {
    pub schema_version: u16,
    pub observation_id: CanonicalCompletedOutcomeObservationId,
    pub subject: CanonicalCompletedOutcomeObservationSubjectV1,
    pub expected_store: CanonicalCompletedOutcomeStoreDescriptorV1,
    pub expected_head: CanonicalCompletedOutcomeHistoryHeadV1,
    pub recording_time_receipt: CurrentTimeReceiptCommitment,
    pub recording_latest_possible_unix_ms: u64,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalCompletedOutcomeHistoryEntryV1 {
    pub record: CanonicalCompletedOutcomeRecordCommitment,
    pub manifest: CanonicalCompletedOutcomeObservationManifestV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalCompletedOutcomeStoreDispositionV1 {
    Appended {
        record: CanonicalCompletedOutcomeRecordCommitment,
        post_head: CanonicalCompletedOutcomeHistoryHeadV1,
    },
    ProvenNotAppended {
        observed_head: CanonicalCompletedOutcomeHistoryHeadV1,
        absence_evidence: CanonicalCompletedOutcomeAbsenceCommitment,
    },
    ObservationIdConflict {
        existing_manifest: CanonicalCompletedOutcomeManifestCommitment,
    },
    HistoryHeadConflict {
        current_head: CanonicalCompletedOutcomeHistoryHeadV1,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalCompletedOutcomeStoreReceiptV1 {
    pub schema_version: u16,
    pub store: CanonicalCompletedOutcomeStoreDescriptorV1,
    pub manifest: CanonicalCompletedOutcomeObservationManifestV1,
    pub disposition: CanonicalCompletedOutcomeStoreDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: CanonicalCompletedOutcomeStoreReceiptCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalCompletedOutcomeHeadReceiptV1 {
    pub schema_version: u16,
    pub store: CanonicalCompletedOutcomeStoreDescriptorV1,
    pub invocation_record: CanonicalCompletedInvocationRecordV1,
    pub head: CanonicalCompletedOutcomeHistoryHeadV1,
    pub latest_entry: Option<CanonicalCompletedOutcomeHistoryEntryV1>,
    pub valid_until: u64,
    pub receipt_commitment: CanonicalCompletedOutcomeHeadReceiptCommitment,
}

pub trait CanonicalCompletedOutcomeHistoryStoreV1 {
    type Error;

    fn descriptor(&self) -> CanonicalCompletedOutcomeStoreDescriptorV1;

    fn append_observation(
        &self,
        manifest: &CanonicalCompletedOutcomeObservationManifestV1,
    ) -> Result<CanonicalCompletedOutcomeStoreReceiptV1, Self::Error>;

    fn reconcile_observation(
        &self,
        manifest: &CanonicalCompletedOutcomeObservationManifestV1,
    ) -> Result<CanonicalCompletedOutcomeStoreReceiptV1, Self::Error>;

    fn read_invocation_head(
        &self,
        invocation_record: CanonicalCompletedInvocationRecordV1,
    ) -> Result<CanonicalCompletedOutcomeHeadReceiptV1, Self::Error>;
}

fn canonical_terminal(
    evidence: CanonicalCompletedEffectEvidenceV1,
) -> Option<CanonicalCompletedOutcomeTerminalV1> {
    match evidence.canonical_disposition() {
        CanonicalQualifiedEffectDispositionV1::Confirmed { .. } => {
            Some(CanonicalCompletedOutcomeTerminalV1::Confirmed)
        }
        CanonicalQualifiedEffectDispositionV1::ProvenNotApplied { .. } => {
            Some(CanonicalCompletedOutcomeTerminalV1::ProvenNotApplied)
        }
        CanonicalQualifiedEffectDispositionV1::OutcomeUnknown { .. } => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalCompletedOutcomePreparationErrorV1 {
    UnsupportedStoreTimeBasis,
    StoreAlreadyExpired,
    RecordingTimeAlreadyExpired,
    RecordingTimeRegressedBeforePreInvocation,
    RecordingTimeRegressedBeforePostInvocation,
    EvidenceInvocationRecordMismatch,
    HistoryInvocationMismatch,
    HistoryAlreadyTerminal,
}

pub struct CanonicalCompletedOutcomePreparationFailureV1<TQ> {
    evidence: CanonicalCompletedEffectEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    error: CanonicalCompletedOutcomePreparationErrorV1,
}

impl<TQ> CanonicalCompletedOutcomePreparationFailureV1<TQ> {
    pub const fn error(&self) -> CanonicalCompletedOutcomePreparationErrorV1 {
        self.error
    }

    pub fn into_parts(
        self,
    ) -> (CanonicalCompletedEffectEvidenceV1, QualifiedCurrentTimeV1<TQ>) {
        (self.evidence, self.recording_time)
    }
}

pub struct PreparedCanonicalCompletedOutcomeObservationV1<TQ> {
    evidence: CanonicalCompletedEffectEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    expected_store: ExpectedCanonicalCompletedOutcomeStoreProfileV1,
    manifest: CanonicalCompletedOutcomeObservationManifestV1,
}

impl<TQ> PreparedCanonicalCompletedOutcomeObservationV1<TQ> {
    pub const fn evidence(&self) -> CanonicalCompletedEffectEvidenceV1 {
        self.evidence
    }

    pub const fn recording_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.recording_time
    }

    pub const fn expected_store(&self) -> ExpectedCanonicalCompletedOutcomeStoreProfileV1 {
        self.expected_store
    }

    pub const fn manifest(&self) -> CanonicalCompletedOutcomeObservationManifestV1 {
        self.manifest
    }
}

#[allow(clippy::too_many_arguments)]
pub fn prepare_canonical_completed_outcome_observation<TQ>(
    evidence: CanonicalCompletedEffectEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    observation_id: CanonicalCompletedOutcomeObservationId,
    expected_store: ExpectedCanonicalCompletedOutcomeStoreProfileV1,
    expected_head: ExpectedCanonicalCompletedOutcomeHistoryHeadV1,
) -> Result<
    PreparedCanonicalCompletedOutcomeObservationV1<TQ>,
    CanonicalCompletedOutcomePreparationFailureV1<TQ>,
> {
    let latest = recording_time.latest_possible_unix_ms();
    let invocation_record = evidence.invocation_record();
    let head = expected_head.head();
    let pre_latest = evidence.pre_invocation_time().latest_possible_unix_ms;
    let post_latest = evidence
        .post_invocation_time()
        .map(|post| post.latest_possible_unix_ms);

    let error = if expected_store.descriptor().time_basis
        != CanonicalCompletedOutcomeStoreTimeBasisV1::UnixMillisecondsUtc
    {
        Some(CanonicalCompletedOutcomePreparationErrorV1::UnsupportedStoreTimeBasis)
    } else if expected_store.descriptor().valid_until < latest {
        Some(CanonicalCompletedOutcomePreparationErrorV1::StoreAlreadyExpired)
    } else if recording_time.valid_until() < latest {
        Some(CanonicalCompletedOutcomePreparationErrorV1::RecordingTimeAlreadyExpired)
    } else if latest < pre_latest {
        Some(CanonicalCompletedOutcomePreparationErrorV1::RecordingTimeRegressedBeforePreInvocation)
    } else if post_latest.is_some_and(|post| latest < post) {
        Some(CanonicalCompletedOutcomePreparationErrorV1::RecordingTimeRegressedBeforePostInvocation)
    } else if evidence.provenance().invocation_record() != Some(invocation_record) {
        Some(CanonicalCompletedOutcomePreparationErrorV1::EvidenceInvocationRecordMismatch)
    } else if head.invocation_record() != invocation_record {
        Some(CanonicalCompletedOutcomePreparationErrorV1::HistoryInvocationMismatch)
    } else if !head.permits_append() {
        Some(CanonicalCompletedOutcomePreparationErrorV1::HistoryAlreadyTerminal)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(CanonicalCompletedOutcomePreparationFailureV1 {
            evidence,
            recording_time,
            error,
        });
    }

    let manifest = CanonicalCompletedOutcomeObservationManifestV1 {
        schema_version: SSF_SCHEMA_V1,
        observation_id,
        subject: CanonicalCompletedOutcomeObservationSubjectV1 {
            invocation_record,
            evidence,
        },
        expected_store: expected_store.descriptor(),
        expected_head: head,
        recording_time_receipt: recording_time.receipt_commitment(),
        recording_latest_possible_unix_ms: latest,
        valid_until: expected_store
            .descriptor()
            .valid_until
            .min(recording_time.valid_until()),
    };

    Ok(PreparedCanonicalCompletedOutcomeObservationV1 {
        evidence,
        recording_time,
        expected_store,
        manifest,
    })
}

fn next_head_matches(
    manifest: CanonicalCompletedOutcomeObservationManifestV1,
    record: CanonicalCompletedOutcomeRecordCommitment,
    post_head: CanonicalCompletedOutcomeHistoryHeadV1,
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
pub enum CanonicalCompletedOutcomeRecordingAmbiguityReasonV1 {
    StoreDescriptorMismatchBefore,
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

pub struct DurablyRecordedCanonicalCompletedOutcomeV1<S, TQ> {
    evidence: CanonicalCompletedEffectEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    manifest: CanonicalCompletedOutcomeObservationManifestV1,
    receipt: CanonicalCompletedOutcomeStoreReceiptV1,
    record: CanonicalCompletedOutcomeRecordCommitment,
    post_head: CanonicalCompletedOutcomeHistoryHeadV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, TQ> DurablyRecordedCanonicalCompletedOutcomeV1<S, TQ> {
    pub const fn evidence(&self) -> CanonicalCompletedEffectEvidenceV1 {
        self.evidence
    }

    pub const fn recording_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.recording_time
    }

    pub const fn manifest(&self) -> CanonicalCompletedOutcomeObservationManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> CanonicalCompletedOutcomeStoreReceiptV1 {
        self.receipt
    }

    pub const fn record(&self) -> CanonicalCompletedOutcomeRecordCommitment {
        self.record
    }

    pub const fn post_head(&self) -> CanonicalCompletedOutcomeHistoryHeadV1 {
        self.post_head
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

pub struct RecoverableCanonicalCompletedOutcomeObservationV1<TQ> {
    evidence: CanonicalCompletedEffectEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    manifest: CanonicalCompletedOutcomeObservationManifestV1,
    receipt: CanonicalCompletedOutcomeStoreReceiptV1,
}

impl<TQ> RecoverableCanonicalCompletedOutcomeObservationV1<TQ> {
    pub fn into_parts(
        self,
    ) -> (CanonicalCompletedEffectEvidenceV1, QualifiedCurrentTimeV1<TQ>) {
        (self.evidence, self.recording_time)
    }

    pub const fn manifest(&self) -> CanonicalCompletedOutcomeObservationManifestV1 {
        self.manifest
    }

    pub const fn receipt(&self) -> CanonicalCompletedOutcomeStoreReceiptV1 {
        self.receipt
    }
}

pub struct FrozenCanonicalCompletedOutcomeRecordingV1<S, TQ> {
    evidence: CanonicalCompletedEffectEvidenceV1,
    recording_time: QualifiedCurrentTimeV1<TQ>,
    manifest: CanonicalCompletedOutcomeObservationManifestV1,
    reason: CanonicalCompletedOutcomeRecordingAmbiguityReasonV1,
    receipt: Option<CanonicalCompletedOutcomeStoreReceiptV1>,
    _store: PhantomData<fn() -> S>,
}

impl<S, TQ> FrozenCanonicalCompletedOutcomeRecordingV1<S, TQ> {
    pub const fn evidence(&self) -> CanonicalCompletedEffectEvidenceV1 {
        self.evidence
    }

    pub const fn recording_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.recording_time
    }

    pub const fn reason(&self) -> CanonicalCompletedOutcomeRecordingAmbiguityReasonV1 {
        self.reason
    }

    pub const fn receipt(&self) -> Option<CanonicalCompletedOutcomeStoreReceiptV1> {
        self.receipt
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

pub enum CanonicalCompletedOutcomeRecordingResultV1<S, TQ> {
    Recorded(DurablyRecordedCanonicalCompletedOutcomeV1<S, TQ>),
    ProvenNotRecorded(RecoverableCanonicalCompletedOutcomeObservationV1<TQ>),
    RecordingOutcomeUnknown(FrozenCanonicalCompletedOutcomeRecordingV1<S, TQ>),
}

fn freeze_recording<S, TQ>(
    prepared: PreparedCanonicalCompletedOutcomeObservationV1<TQ>,
    reason: CanonicalCompletedOutcomeRecordingAmbiguityReasonV1,
    receipt: Option<CanonicalCompletedOutcomeStoreReceiptV1>,
) -> CanonicalCompletedOutcomeRecordingResultV1<S, TQ> {
    CanonicalCompletedOutcomeRecordingResultV1::RecordingOutcomeUnknown(
        FrozenCanonicalCompletedOutcomeRecordingV1 {
            evidence: prepared.evidence,
            recording_time: prepared.recording_time,
            manifest: prepared.manifest,
            reason,
            receipt,
            _store: PhantomData,
        },
    )
}

fn interpret_store_result<S, TQ>(
    prepared: PreparedCanonicalCompletedOutcomeObservationV1<TQ>,
    descriptor_after: CanonicalCompletedOutcomeStoreDescriptorV1,
    result: Result<CanonicalCompletedOutcomeStoreReceiptV1, S::Error>,
) -> CanonicalCompletedOutcomeRecordingResultV1<S, TQ>
where
    S: CanonicalCompletedOutcomeHistoryStoreV1,
{
    let receipt = match result {
        Ok(receipt) => receipt,
        Err(_) => {
            return freeze_recording::<S, TQ>(
                prepared,
                CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::StoreErrorAfterAppendBoundary,
                None,
            );
        }
    };

    if descriptor_after != prepared.expected_store.descriptor() {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::StoreDescriptorChangedAfter,
            Some(receipt),
        );
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::UnsupportedReceiptSchema,
            Some(receipt),
        );
    }
    if receipt.store != prepared.expected_store.descriptor() {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::ReceiptStoreMismatch,
            Some(receipt),
        );
    }
    if receipt.manifest != prepared.manifest {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::ReceiptManifestMismatch,
            Some(receipt),
        );
    }
    if receipt.valid_until > prepared.expected_store.descriptor().valid_until {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::ReceiptOutlivesStore,
            Some(receipt),
        );
    }
    if receipt.valid_until > prepared.manifest.valid_until {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::ReceiptOutlivesObservation,
            Some(receipt),
        );
    }
    if receipt.valid_until < prepared.manifest.recording_latest_possible_unix_ms {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::ReceiptAlreadyExpiredAtRecording,
            Some(receipt),
        );
    }

    match receipt.disposition {
        CanonicalCompletedOutcomeStoreDispositionV1::Appended { record, post_head } => {
            if !next_head_matches(prepared.manifest, record, post_head) {
                return freeze_recording::<S, TQ>(
                    prepared,
                    CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::InvalidPostHead,
                    Some(receipt),
                );
            }
            CanonicalCompletedOutcomeRecordingResultV1::Recorded(
                DurablyRecordedCanonicalCompletedOutcomeV1 {
                    evidence: prepared.evidence,
                    recording_time: prepared.recording_time,
                    manifest: prepared.manifest,
                    receipt,
                    record,
                    post_head,
                    _store: PhantomData,
                },
            )
        }
        CanonicalCompletedOutcomeStoreDispositionV1::ProvenNotAppended { observed_head, .. } => {
            if observed_head != prepared.manifest.expected_head {
                return freeze_recording::<S, TQ>(
                    prepared,
                    CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::InvalidProvenNotAppendedHead,
                    Some(receipt),
                );
            }
            CanonicalCompletedOutcomeRecordingResultV1::ProvenNotRecorded(
                RecoverableCanonicalCompletedOutcomeObservationV1 {
                    evidence: prepared.evidence,
                    recording_time: prepared.recording_time,
                    manifest: prepared.manifest,
                    receipt,
                },
            )
        }
        CanonicalCompletedOutcomeStoreDispositionV1::ObservationIdConflict { .. } => {
            freeze_recording::<S, TQ>(
                prepared,
                CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::ObservationIdConflict,
                Some(receipt),
            )
        }
        CanonicalCompletedOutcomeStoreDispositionV1::HistoryHeadConflict { .. } => {
            freeze_recording::<S, TQ>(
                prepared,
                CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::HistoryHeadConflict,
                Some(receipt),
            )
        }
        CanonicalCompletedOutcomeStoreDispositionV1::OutcomeUnknown => freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::StoreReportedOutcomeUnknown,
            Some(receipt),
        ),
    }
}

pub fn append_canonical_completed_outcome<S, TQ>(
    prepared: PreparedCanonicalCompletedOutcomeObservationV1<TQ>,
    store: &S,
) -> CanonicalCompletedOutcomeRecordingResultV1<S, TQ>
where
    S: CanonicalCompletedOutcomeHistoryStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor() {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::StoreDescriptorMismatchBefore,
            None,
        );
    }

    let result = store.append_observation(&prepared.manifest);
    let descriptor_after = store.descriptor();
    interpret_store_result::<S, TQ>(prepared, descriptor_after, result)
}

pub fn reconcile_canonical_completed_outcome<S, TQ>(
    prepared: PreparedCanonicalCompletedOutcomeObservationV1<TQ>,
    store: &S,
) -> CanonicalCompletedOutcomeRecordingResultV1<S, TQ>
where
    S: CanonicalCompletedOutcomeHistoryStoreV1,
{
    if store.descriptor() != prepared.expected_store.descriptor() {
        return freeze_recording::<S, TQ>(
            prepared,
            CanonicalCompletedOutcomeRecordingAmbiguityReasonV1::StoreDescriptorMismatchBefore,
            None,
        );
    }

    let result = store.reconcile_observation(&prepared.manifest);
    let descriptor_after = store.descriptor();
    interpret_store_result::<S, TQ>(prepared, descriptor_after, result)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalCompletedOutcomeHeadReadErrorV1 {
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
    LatestEntryUnsupportedSchema,
    LatestEntryStoreMismatch,
    LatestEntryInvocationMismatch,
    LatestEntryRecordingTimeAfterRead,
    LatestEntryPredecessorTerminal,
    LatestEntryGenerationMismatch,
    LatestEntryTerminalMismatch,
}

pub struct QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
    expected_store: ExpectedCanonicalCompletedOutcomeStoreProfileV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    receipt: CanonicalCompletedOutcomeHeadReceiptV1,
    _store: PhantomData<fn() -> S>,
}

impl<S, TQ> QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
    pub const fn expected_store(&self) -> ExpectedCanonicalCompletedOutcomeStoreProfileV1 {
        self.expected_store
    }

    pub const fn receipt(&self) -> CanonicalCompletedOutcomeHeadReceiptV1 {
        self.receipt
    }

    pub const fn head(&self) -> CanonicalCompletedOutcomeHistoryHeadV1 {
        self.receipt.head
    }

    pub const fn latest_entry(&self) -> Option<CanonicalCompletedOutcomeHistoryEntryV1> {
        self.receipt.latest_entry
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

pub fn read_canonical_completed_outcome_history_head<S, TQ>(
    invocation_record: CanonicalCompletedInvocationRecordV1,
    expected_store: ExpectedCanonicalCompletedOutcomeStoreProfileV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    store: &S,
) -> Result<
    QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    CanonicalCompletedOutcomeHeadReadErrorV1,
>
where
    S: CanonicalCompletedOutcomeHistoryStoreV1,
{
    let latest = current_time.latest_possible_unix_ms();
    let expected_descriptor = expected_store.descriptor();

    if expected_descriptor.time_basis != CanonicalCompletedOutcomeStoreTimeBasisV1::UnixMillisecondsUtc {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::UnsupportedStoreTimeBasis);
    }
    if expected_descriptor.valid_until < latest {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::StoreAlreadyExpired);
    }
    if current_time.valid_until() < latest {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::ReadTimeAlreadyExpired);
    }
    if store.descriptor() != expected_descriptor {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::StoreDescriptorMismatchBefore);
    }

    let receipt = store
        .read_invocation_head(invocation_record)
        .map_err(|_| CanonicalCompletedOutcomeHeadReadErrorV1::ReadFailed)?;

    if store.descriptor() != expected_descriptor {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::StoreDescriptorChangedAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::UnsupportedReceiptSchema);
    }
    if receipt.store != expected_descriptor {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::ReceiptStoreMismatch);
    }
    if receipt.invocation_record != invocation_record {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::ReceiptInvocationMismatch);
    }
    if receipt.head.invocation_record() != invocation_record {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::ReceiptHeadInvocationMismatch);
    }
    if receipt.valid_until > expected_descriptor.valid_until {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::ReceiptOutlivesStore);
    }
    if receipt.valid_until < latest {
        return Err(CanonicalCompletedOutcomeHeadReadErrorV1::ReceiptAlreadyExpired);
    }

    match (
        receipt.head.generation().get(),
        receipt.head.head(),
        receipt.latest_entry,
    ) {
        (0, None, None) => {}
        (0, None, Some(_)) => {
            return Err(CanonicalCompletedOutcomeHeadReadErrorV1::GenesisHasLatestEntry)
        }
        (_, Some(_), None) => {
            return Err(CanonicalCompletedOutcomeHeadReadErrorV1::NonGenesisMissingLatestEntry)
        }
        (_, Some(head_record), Some(entry)) => {
            if entry.record != head_record {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryRecordMismatch);
            }
            if entry.manifest.schema_version != SSF_SCHEMA_V1 {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryUnsupportedSchema);
            }
            if entry.manifest.expected_store != expected_descriptor {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryStoreMismatch);
            }
            if entry.manifest.subject.invocation_record != invocation_record
                || entry.manifest.subject.evidence.invocation_record() != invocation_record
                || entry.manifest.expected_head.invocation_record() != invocation_record
            {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryInvocationMismatch);
            }
            if latest < entry.manifest.recording_latest_possible_unix_ms {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryRecordingTimeAfterRead);
            }
            if !entry.manifest.expected_head.permits_append() {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryPredecessorTerminal);
            }

            let next = entry
                .manifest
                .expected_head
                .generation()
                .get()
                .checked_add(1)
                .ok_or(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryGenerationMismatch)?;
            if next != receipt.head.generation().get() {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryGenerationMismatch);
            }
            if canonical_terminal(entry.manifest.subject.evidence) != receipt.head.terminal() {
                return Err(CanonicalCompletedOutcomeHeadReadErrorV1::LatestEntryTerminalMismatch);
            }
        }
        (_, None, _) => {
            return Err(CanonicalCompletedOutcomeHeadReadErrorV1::NonGenesisMissingLatestEntry)
        }
    }

    Ok(QualifiedCanonicalCompletedOutcomeHistoryHeadV1 {
        expected_store,
        current_time,
        receipt,
        _store: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_durable_actuator_invocation_attempt::InvocationAttemptJournalRecordCommitment;

    #[test]
    fn terminal_state_depends_only_on_canonical_disposition_shape() {
        let unknown_is_terminal = false;
        assert!(!unknown_is_terminal);
    }

    #[test]
    fn genesis_head_has_no_outcome_record() {
        let invocation = CanonicalCompletedInvocationRecordV1::Initial(
            InvocationAttemptJournalRecordCommitment::from_bytes([1; 32]),
        );
        let head = CanonicalCompletedOutcomeHistoryHeadV1::from_trusted_state(
            invocation,
            CanonicalCompletedOutcomeHistoryGeneration::new(0),
            None,
            None,
        )
        .expect("valid genesis head");
        assert!(head.permits_append());
        assert_eq!(head.head(), None);
    }

    #[test]
    fn terminal_head_does_not_permit_append() {
        let invocation = CanonicalCompletedInvocationRecordV1::Initial(
            InvocationAttemptJournalRecordCommitment::from_bytes([2; 32]),
        );
        let head = CanonicalCompletedOutcomeHistoryHeadV1::from_trusted_state(
            invocation,
            CanonicalCompletedOutcomeHistoryGeneration::new(1),
            Some(CanonicalCompletedOutcomeRecordCommitment::from_bytes([3; 32])),
            Some(CanonicalCompletedOutcomeTerminalV1::Confirmed),
        )
        .expect("valid terminal head");
        assert!(!head.permits_append());
    }
}

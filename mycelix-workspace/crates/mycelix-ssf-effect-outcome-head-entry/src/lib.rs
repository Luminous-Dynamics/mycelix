// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact durable outcome-head record/manifest pairing for SSF replay evidence.
//!
//! `mycelix-ssf-durable-effect-outcome-history` qualifies a durable outcome
//! history head and returns optional latest-manifest metadata. This crate adds a
//! narrower second read from the same concrete store type that must pair the
//! exact durable head record with the exact observation manifest describing it.
//! Replay policy should consume this stronger typestate rather than infer that a
//! compatible manifest necessarily names the durable head record.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_durable_actuator_invocation_attempt::InvocationAttemptJournalRecordCommitment;
use mycelix_ssf_durable_effect_outcome_history::{
    EffectOutcomeHistoryHeadReceiptCommitment, EffectOutcomeHistoryHeadV1,
    EffectOutcomeHistoryStoreV1, EffectOutcomeObservationManifestV1,
    EffectOutcomeRecordCommitment, EffectOutcomeStoreDescriptorV1,
    EffectOutcomeStoreTimeBasisV1, QualifiedEffectOutcomeHistoryHeadV1,
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

digest_type!(EffectOutcomeHeadEntryReceiptCommitment);

/// Explicit store-produced pairing of one durable head record and the
/// observation manifest that describes that record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectOutcomeHeadEntryReceiptV1 {
    pub schema_version: u16,
    pub store: EffectOutcomeStoreDescriptorV1,
    pub journal_record: InvocationAttemptJournalRecordCommitment,
    pub qualified_head_receipt: EffectOutcomeHistoryHeadReceiptCommitment,
    pub head: EffectOutcomeHistoryHeadV1,
    pub record: EffectOutcomeRecordCommitment,
    pub manifest: EffectOutcomeObservationManifestV1,
    pub valid_until: u64,
    pub receipt_commitment: EffectOutcomeHeadEntryReceiptCommitment,
}

/// Extension implemented by the same concrete outcome-history store that can
/// already provide a qualified history-head read.
pub trait EffectOutcomeHeadEntryStoreV1: EffectOutcomeHistoryStoreV1 {
    fn read_paired_head_entry(
        &self,
        journal_record: InvocationAttemptJournalRecordCommitment,
        qualified_head_receipt: EffectOutcomeHistoryHeadReceiptCommitment,
    ) -> Result<EffectOutcomeHeadEntryReceiptV1, <Self as EffectOutcomeHistoryStoreV1>::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectOutcomeHeadEntryQualificationErrorV1 {
    HistoryHeadIsGenesis,
    HistoryHeadMissingRecord,
    HistoryHeadMissingLatestManifest,
    UnsupportedStoreTimeBasis,
    StoreAlreadyExpired,
    HistoryReadAlreadyExpired,
    CurrentTimeAlreadyExpired,
    ManifestAlreadyExpired,
    StoreDescriptorMismatchBefore,
    StoreReadError,
    StoreDescriptorChangedAfter,
    UnsupportedEntryReceiptSchema,
    EntryStoreMismatch,
    EntryJournalMismatch,
    EntryQualifiedHeadReceiptMismatch,
    EntryHeadMismatch,
    EntryRecordMismatch,
    EntryManifestMismatch,
    UnsupportedManifestSchema,
    ManifestJournalMismatch,
    ManifestStoreMismatch,
    ManifestPredecessorJournalMismatch,
    ManifestGenerationOverflow,
    ManifestDoesNotExtendHead,
    ManifestTerminalStateMismatch,
    EntryReceiptOutlivesStore,
    EntryReceiptOutlivesHistoryRead,
    EntryReceiptOutlivesManifest,
    EntryReceiptAlreadyExpired,
}

pub struct ExactEffectOutcomeHeadEntryQualificationFailureV1<S, TQ> {
    qualified_head: QualifiedEffectOutcomeHistoryHeadV1<S>,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: EffectOutcomeHeadEntryQualificationErrorV1,
}

impl<S, TQ> ExactEffectOutcomeHeadEntryQualificationFailureV1<S, TQ> {
    pub const fn error(&self) -> EffectOutcomeHeadEntryQualificationErrorV1 {
        self.error
    }

    pub fn into_parts(
        self,
    ) -> (
        QualifiedEffectOutcomeHistoryHeadV1<S>,
        QualifiedCurrentTimeV1<TQ>,
    ) {
        (self.qualified_head, self.current_time)
    }
}

/// Opaque evidence that the same concrete outcome store explicitly paired the
/// exact durable head record with the exact latest manifest used downstream.
pub struct QualifiedExactEffectOutcomeHeadEntryV1<S, TQ> {
    qualified_head: QualifiedEffectOutcomeHistoryHeadV1<S>,
    current_time: QualifiedCurrentTimeV1<TQ>,
    entry_receipt: EffectOutcomeHeadEntryReceiptV1,
    valid_until: u64,
}

impl<S, TQ> QualifiedExactEffectOutcomeHeadEntryV1<S, TQ> {
    pub const fn qualified_head(&self) -> &QualifiedEffectOutcomeHistoryHeadV1<S> {
        &self.qualified_head
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn entry_receipt(&self) -> EffectOutcomeHeadEntryReceiptV1 {
        self.entry_receipt
    }

    pub const fn head(&self) -> EffectOutcomeHistoryHeadV1 {
        self.entry_receipt.head
    }

    pub const fn record(&self) -> EffectOutcomeRecordCommitment {
        self.entry_receipt.record
    }

    pub const fn manifest(&self) -> EffectOutcomeObservationManifestV1 {
        self.entry_receipt.manifest
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

fn head_record_is_exact(
    head: EffectOutcomeHistoryHeadV1,
    record: EffectOutcomeRecordCommitment,
) -> bool {
    head.head() == Some(record)
}

fn manifest_extends_exact_head(
    manifest: EffectOutcomeObservationManifestV1,
    head: EffectOutcomeHistoryHeadV1,
) -> Result<bool, EffectOutcomeHeadEntryQualificationErrorV1> {
    if manifest.expected_head.journal_record() != head.journal_record() {
        return Err(
            EffectOutcomeHeadEntryQualificationErrorV1::ManifestPredecessorJournalMismatch,
        );
    }
    let expected_generation = manifest
        .expected_head
        .generation()
        .get()
        .checked_add(1)
        .ok_or(EffectOutcomeHeadEntryQualificationErrorV1::ManifestGenerationOverflow)?;

    Ok(expected_generation == head.generation().get()
        && manifest.subject.journal_record == head.journal_record())
}

fn failure<S, TQ>(
    qualified_head: QualifiedEffectOutcomeHistoryHeadV1<S>,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: EffectOutcomeHeadEntryQualificationErrorV1,
) -> ExactEffectOutcomeHeadEntryQualificationFailureV1<S, TQ> {
    ExactEffectOutcomeHeadEntryQualificationFailureV1 {
        qualified_head,
        current_time,
        error,
    }
}

pub fn qualify_exact_effect_outcome_head_entry<S, TQ>(
    qualified_head: QualifiedEffectOutcomeHistoryHeadV1<S>,
    current_time: QualifiedCurrentTimeV1<TQ>,
    store: &S,
) -> Result<
    QualifiedExactEffectOutcomeHeadEntryV1<S, TQ>,
    ExactEffectOutcomeHeadEntryQualificationFailureV1<S, TQ>,
>
where
    S: EffectOutcomeHeadEntryStoreV1,
{
    let head_read = qualified_head.receipt();
    let latest_now = current_time.latest_possible_unix_ms();

    if head_read.head.generation().get() == 0 {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::HistoryHeadIsGenesis,
        ));
    }
    let expected_record = match head_read.head.head() {
        Some(record) => record,
        None => {
            return Err(failure(
                qualified_head,
                current_time,
                EffectOutcomeHeadEntryQualificationErrorV1::HistoryHeadMissingRecord,
            ));
        }
    };
    let expected_manifest = match head_read.latest_manifest {
        Some(manifest) => manifest,
        None => {
            return Err(failure(
                qualified_head,
                current_time,
                EffectOutcomeHeadEntryQualificationErrorV1::HistoryHeadMissingLatestManifest,
            ));
        }
    };

    if head_read.store.time_basis != EffectOutcomeStoreTimeBasisV1::UnixMillisecondsUtc {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::UnsupportedStoreTimeBasis,
        ));
    }
    if head_read.store.valid_until < latest_now {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::StoreAlreadyExpired,
        ));
    }
    if head_read.valid_until < latest_now {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::HistoryReadAlreadyExpired,
        ));
    }
    if current_time.valid_until() < latest_now {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::CurrentTimeAlreadyExpired,
        ));
    }
    if expected_manifest.valid_until < latest_now {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::ManifestAlreadyExpired,
        ));
    }
    if store.descriptor() != head_read.store {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::StoreDescriptorMismatchBefore,
        ));
    }

    let entry = match store.read_paired_head_entry(
        head_read.journal_record,
        head_read.receipt_commitment,
    ) {
        Ok(entry) => entry,
        Err(_) => {
            return Err(failure(
                qualified_head,
                current_time,
                EffectOutcomeHeadEntryQualificationErrorV1::StoreReadError,
            ));
        }
    };

    if store.descriptor() != head_read.store {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::StoreDescriptorChangedAfter,
        ));
    }
    if entry.schema_version != SSF_SCHEMA_V1 {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::UnsupportedEntryReceiptSchema,
        ));
    }
    if entry.store != head_read.store {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryStoreMismatch,
        ));
    }
    if entry.journal_record != head_read.journal_record {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryJournalMismatch,
        ));
    }
    if entry.qualified_head_receipt != head_read.receipt_commitment {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryQualifiedHeadReceiptMismatch,
        ));
    }
    if entry.head != head_read.head {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryHeadMismatch,
        ));
    }
    if entry.record != expected_record || !head_record_is_exact(entry.head, entry.record) {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryRecordMismatch,
        ));
    }
    if entry.manifest != expected_manifest {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryManifestMismatch,
        ));
    }
    if entry.manifest.schema_version != SSF_SCHEMA_V1 {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::UnsupportedManifestSchema,
        ));
    }
    if entry.manifest.subject.journal_record != head_read.journal_record {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::ManifestJournalMismatch,
        ));
    }
    if entry.manifest.expected_store != head_read.store {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::ManifestStoreMismatch,
        ));
    }
    match manifest_extends_exact_head(entry.manifest, entry.head) {
        Ok(true) => {}
        Ok(false) => {
            return Err(failure(
                qualified_head,
                current_time,
                EffectOutcomeHeadEntryQualificationErrorV1::ManifestDoesNotExtendHead,
            ));
        }
        Err(error) => return Err(failure(qualified_head, current_time, error)),
    }
    if entry.manifest.subject.state.terminal() != entry.head.terminal() {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::ManifestTerminalStateMismatch,
        ));
    }
    if entry.valid_until > head_read.store.valid_until {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryReceiptOutlivesStore,
        ));
    }
    if entry.valid_until > head_read.valid_until {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryReceiptOutlivesHistoryRead,
        ));
    }
    if entry.valid_until > entry.manifest.valid_until {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryReceiptOutlivesManifest,
        ));
    }
    if entry.valid_until < latest_now {
        return Err(failure(
            qualified_head,
            current_time,
            EffectOutcomeHeadEntryQualificationErrorV1::EntryReceiptAlreadyExpired,
        ));
    }

    let valid_until = head_read
        .store
        .valid_until
        .min(head_read.valid_until)
        .min(entry.manifest.valid_until)
        .min(entry.valid_until)
        .min(current_time.valid_until());

    Ok(QualifiedExactEffectOutcomeHeadEntryV1 {
        qualified_head,
        current_time,
        entry_receipt: entry,
        valid_until,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_durable_effect_outcome_history::EffectOutcomeHistoryGeneration;

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
    fn head_record_match_is_exact() {
        let record = outcome_record(2);
        let head = EffectOutcomeHistoryHeadV1::from_trusted_state(
            journal_record(1),
            EffectOutcomeHistoryGeneration::new(1),
            Some(record),
            None,
        )
        .expect("valid head");

        assert!(head_record_is_exact(head, record));
        assert!(!head_record_is_exact(head, outcome_record(3)));
    }

    #[test]
    fn genesis_cannot_supply_a_paired_durable_record() {
        let head = EffectOutcomeHistoryHeadV1::from_trusted_state(
            journal_record(1),
            EffectOutcomeHistoryGeneration::new(0),
            None,
            None,
        )
        .expect("valid genesis");

        assert_eq!(head.head(), None);
    }
}

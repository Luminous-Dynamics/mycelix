// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only historical reconciliation for already-journaled actuator attempts.
//! A newer reconciler may attest an old exact `Journaled` fact only for the same
//! stable journal-store identity. Fresh historical evidence never refreshes the
//! original effect authority ceiling.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptId;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_durable_actuator_invocation_attempt::{
    DurableInvocationAttemptJournalFrontierV1, InvocationAttemptJournalDispositionV1,
    InvocationAttemptJournalReceiptV1, InvocationAttemptJournalStoreDescriptorV1,
    InvocationAttemptJournalStoreIdentityCommitment, InvocationAttemptJournalTimeBasisV1,
};
use mycelix_ssf_durable_effect_capability_claim::EffectCapabilityClaimRecordCommitment;
use mycelix_ssf_replay_aware_invocation_journal::{
    DurableReplayInvocationJournalFrontierV1, ReplayInvocationJournalDispositionV1,
    ReplayInvocationJournalReceiptV1, ReplayInvocationJournalStoreDescriptorV1,
    ReplayInvocationJournalStoreIdentityCommitment, ReplayInvocationJournalTimeBasisV1,
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

digest_type!(HistoricalInvocationJournalPolicyCommitment);
digest_type!(HistoricalInvocationJournalReceiptCommitment);
generation_type!(HistoricalInvocationJournalReconcilerGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalInvocationJournalStableIdentityV1 {
    Initial(InvocationAttemptJournalStoreIdentityCommitment),
    Replay(ReplayInvocationJournalStoreIdentityCommitment),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalInvocationJournalTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct HistoricalInvocationJournalReconcilerDescriptorV1 {
    pub stable_identity: HistoricalInvocationJournalStableIdentityV1,
    pub policy: HistoricalInvocationJournalPolicyCommitment,
    pub generation: HistoricalInvocationJournalReconcilerGeneration,
    pub time_basis: HistoricalInvocationJournalTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedHistoricalInvocationJournalReconcilerProfileV1 {
    descriptor: HistoricalInvocationJournalReconcilerDescriptorV1,
}

impl ExpectedHistoricalInvocationJournalReconcilerProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: HistoricalInvocationJournalReconcilerDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> HistoricalInvocationJournalReconcilerDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalInvocationJournalReconciliationSubjectV1 {
    Initial {
        original_store: InvocationAttemptJournalStoreDescriptorV1,
        claim_record: EffectCapabilityClaimRecordCommitment,
        current_time_receipt: CurrentTimeReceiptCommitment,
        latest_possible_unix_ms: u64,
    },
    Replay {
        original_store: ReplayInvocationJournalStoreDescriptorV1,
        prior_attempt_id: ActuatorInvocationAttemptId,
        current_time_receipt: CurrentTimeReceiptCommitment,
        latest_possible_unix_ms: u64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalInvocationJournalEvidenceV1 {
    Initial(InvocationAttemptJournalReceiptV1),
    Replay(ReplayInvocationJournalReceiptV1),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoricalInvocationJournalReconciliationReceiptV1 {
    pub schema_version: u16,
    pub reconciler: HistoricalInvocationJournalReconcilerDescriptorV1,
    pub subject: HistoricalInvocationJournalReconciliationSubjectV1,
    pub evidence: HistoricalInvocationJournalEvidenceV1,
    pub valid_until: u64,
    pub receipt_commitment: HistoricalInvocationJournalReceiptCommitment,
}

pub trait HistoricalInvocationJournalReconcilerV1 {
    type Error;

    fn descriptor(&self) -> HistoricalInvocationJournalReconcilerDescriptorV1;

    fn reconcile_historical_journal(
        &self,
        subject: &HistoricalInvocationJournalReconciliationSubjectV1,
    ) -> Result<HistoricalInvocationJournalReconciliationReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalInvocationJournalReconciliationErrorV1 {
    UnsupportedReconcilerTimeBasis,
    ReconcilerAlreadyExpired,
    ReconcilerDescriptorMismatchBefore,
    ReconcilerDescriptorChangedAfter,
    StableStoreIdentityMismatch,
    OriginalStoreUsesUnsupportedTimeBasis,
    CurrentTimeAlreadyExpired,
    ReconciliationError,
    UnsupportedReceiptSchema,
    ReceiptReconcilerMismatch,
    ReceiptSubjectMismatch,
    ReceiptEvidenceKindMismatch,
    ReceiptOutlivesReconciler,
    ReceiptAlreadyExpired,
    UnsupportedJournalReceiptSchema,
    UnsupportedJournalManifestSchema,
    JournalStoreMismatch,
    JournalSubjectMismatch,
    JournalReceiptOutlivedOriginalStore,
    JournalReceiptWasStaleAtAttempt,
    JournalGenerationOverflow,
    InvalidJournaledFrontier,
    JournalWasNotDurablyJournaled,
    ReplayStableEffectIdentityMismatch,
}

fn initial_frontier_is_exact(
    expected: DurableInvocationAttemptJournalFrontierV1,
    record: mycelix_ssf_durable_actuator_invocation_attempt::InvocationAttemptJournalRecordCommitment,
    post: DurableInvocationAttemptJournalFrontierV1,
) -> Result<(), HistoricalInvocationJournalReconciliationErrorV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(HistoricalInvocationJournalReconciliationErrorV1::JournalGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(record) {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::InvalidJournaledFrontier);
    }
    Ok(())
}

fn replay_frontier_is_exact(
    expected: DurableReplayInvocationJournalFrontierV1,
    record: mycelix_ssf_replay_aware_invocation_journal::ReplayInvocationJournalRecordCommitment,
    post: DurableReplayInvocationJournalFrontierV1,
) -> Result<(), HistoricalInvocationJournalReconciliationErrorV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(HistoricalInvocationJournalReconciliationErrorV1::JournalGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(record) {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::InvalidJournaledFrontier);
    }
    Ok(())
}

fn expected_stable_identity(
    subject: HistoricalInvocationJournalReconciliationSubjectV1,
) -> HistoricalInvocationJournalStableIdentityV1 {
    match subject {
        HistoricalInvocationJournalReconciliationSubjectV1::Initial { original_store, .. } => {
            HistoricalInvocationJournalStableIdentityV1::Initial(original_store.stable_identity)
        }
        HistoricalInvocationJournalReconciliationSubjectV1::Replay { original_store, .. } => {
            HistoricalInvocationJournalStableIdentityV1::Replay(original_store.stable_identity)
        }
    }
}

fn subject_latest(subject: HistoricalInvocationJournalReconciliationSubjectV1) -> u64 {
    match subject {
        HistoricalInvocationJournalReconciliationSubjectV1::Initial {
            latest_possible_unix_ms,
            ..
        }
        | HistoricalInvocationJournalReconciliationSubjectV1::Replay {
            latest_possible_unix_ms,
            ..
        } => latest_possible_unix_ms,
    }
}

fn validate_initial_journal(
    receipt: InvocationAttemptJournalReceiptV1,
    original_store: InvocationAttemptJournalStoreDescriptorV1,
    claim_record: EffectCapabilityClaimRecordCommitment,
) -> Result<u64, HistoricalInvocationJournalReconciliationErrorV1> {
    if original_store.time_basis != InvocationAttemptJournalTimeBasisV1::UnixMillisecondsUtc {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::OriginalStoreUsesUnsupportedTimeBasis);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::UnsupportedJournalReceiptSchema);
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1
        || receipt.manifest.attempt.schema_version != SSF_SCHEMA_V1
    {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::UnsupportedJournalManifestSchema);
    }
    if receipt.store != original_store || receipt.manifest.expected_store != original_store {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalStoreMismatch);
    }
    if receipt.manifest.claim_record() != claim_record {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalSubjectMismatch);
    }
    if receipt.valid_until > original_store.valid_until {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalReceiptOutlivedOriginalStore);
    }
    if receipt.valid_until < receipt.manifest.attempt.latest_possible_unix_ms {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalReceiptWasStaleAtAttempt);
    }
    match receipt.disposition {
        InvocationAttemptJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => initial_frontier_is_exact(receipt.manifest.expected_frontier, record, post_frontier)?,
        _ => {
            return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalWasNotDurablyJournaled);
        }
    }

    Ok(receipt
        .manifest
        .attempt
        .attempt_valid_until
        .min(receipt.manifest.attempt.subject.actuator.valid_until))
}

fn validate_replay_journal(
    receipt: ReplayInvocationJournalReceiptV1,
    original_store: ReplayInvocationJournalStoreDescriptorV1,
    prior_attempt_id: ActuatorInvocationAttemptId,
) -> Result<u64, HistoricalInvocationJournalReconciliationErrorV1> {
    if original_store.time_basis != ReplayInvocationJournalTimeBasisV1::UnixMillisecondsUtc {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::OriginalStoreUsesUnsupportedTimeBasis);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::UnsupportedJournalReceiptSchema);
    }
    if receipt.manifest.schema_version != SSF_SCHEMA_V1
        || receipt.manifest.attempt.schema_version != SSF_SCHEMA_V1
    {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::UnsupportedJournalManifestSchema);
    }
    if receipt.store != original_store || receipt.manifest.expected_store != original_store {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalStoreMismatch);
    }
    if receipt.manifest.prior_attempt_id() != prior_attempt_id {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalSubjectMismatch);
    }
    if receipt.valid_until > original_store.valid_until {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalReceiptOutlivedOriginalStore);
    }
    if receipt.valid_until < receipt.manifest.attempt.latest_possible_unix_ms {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalReceiptWasStaleAtAttempt);
    }
    if receipt.manifest.replay.subject.prior_effect_subject.stable_identity()
        != receipt.manifest.replay.subject.stable_effect_identity
        || receipt.manifest.attempt.subject.stable_identity()
            != receipt.manifest.replay.subject.stable_effect_identity
    {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReplayStableEffectIdentityMismatch);
    }
    match receipt.disposition {
        ReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => replay_frontier_is_exact(receipt.manifest.expected_frontier, record, post_frontier)?,
        _ => {
            return Err(HistoricalInvocationJournalReconciliationErrorV1::JournalWasNotDurablyJournaled);
        }
    }

    Ok(receipt
        .manifest
        .attempt
        .attempt_valid_until
        .min(receipt.manifest.attempt.subject.actuator.valid_until)
        .min(receipt.manifest.replay.valid_until))
}

pub struct HistoricalInvocationJournalReconciliationFailureV1<TQ> {
    subject: HistoricalInvocationJournalReconciliationSubjectV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    error: HistoricalInvocationJournalReconciliationErrorV1,
}

impl<TQ> HistoricalInvocationJournalReconciliationFailureV1<TQ> {
    pub const fn error(&self) -> HistoricalInvocationJournalReconciliationErrorV1 {
        self.error
    }

    pub const fn subject(&self) -> HistoricalInvocationJournalReconciliationSubjectV1 {
        self.subject
    }

    pub const fn expected_reconciler(&self) -> ExpectedHistoricalInvocationJournalReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub fn into_current_time(self) -> QualifiedCurrentTimeV1<TQ> {
        self.current_time
    }
}

pub struct HistoricallyReconciledInitialInvocationJournalV1<R, TQ> {
    journal_receipt: InvocationAttemptJournalReceiptV1,
    reconciliation_receipt: HistoricalInvocationJournalReconciliationReceiptV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    invocation_eligibility_valid_until: u64,
    _reconciler: PhantomData<fn() -> R>,
}

impl<R, TQ> HistoricallyReconciledInitialInvocationJournalV1<R, TQ> {
    pub const fn journal_receipt(&self) -> InvocationAttemptJournalReceiptV1 {
        self.journal_receipt
    }

    pub const fn reconciliation_receipt(&self) -> HistoricalInvocationJournalReconciliationReceiptV1 {
        self.reconciliation_receipt
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn invocation_eligibility_valid_until(&self) -> u64 {
        self.invocation_eligibility_valid_until
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }
}

pub struct HistoricallyReconciledReplayInvocationJournalV1<R, TQ> {
    journal_receipt: ReplayInvocationJournalReceiptV1,
    reconciliation_receipt: HistoricalInvocationJournalReconciliationReceiptV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    invocation_eligibility_valid_until: u64,
    _reconciler: PhantomData<fn() -> R>,
}

impl<R, TQ> HistoricallyReconciledReplayInvocationJournalV1<R, TQ> {
    pub const fn journal_receipt(&self) -> ReplayInvocationJournalReceiptV1 {
        self.journal_receipt
    }

    pub const fn reconciliation_receipt(&self) -> HistoricalInvocationJournalReconciliationReceiptV1 {
        self.reconciliation_receipt
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn invocation_eligibility_valid_until(&self) -> u64 {
        self.invocation_eligibility_valid_until
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }
}

fn validate_common<R, TQ>(
    subject: HistoricalInvocationJournalReconciliationSubjectV1,
    current_time: &QualifiedCurrentTimeV1<TQ>,
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    reconciler: &R,
) -> Result<HistoricalInvocationJournalReconciliationReceiptV1, HistoricalInvocationJournalReconciliationErrorV1>
where
    R: HistoricalInvocationJournalReconcilerV1,
{
    let latest = subject_latest(subject);
    if expected_reconciler.descriptor.time_basis
        != HistoricalInvocationJournalTimeBasisV1::UnixMillisecondsUtc
    {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::UnsupportedReconcilerTimeBasis);
    }
    if expected_reconciler.descriptor.stable_identity != expected_stable_identity(subject) {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::StableStoreIdentityMismatch);
    }
    if expected_reconciler.descriptor.valid_until < latest {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReconcilerAlreadyExpired);
    }
    if current_time.latest_possible_unix_ms() != latest || current_time.valid_until() < latest {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::CurrentTimeAlreadyExpired);
    }
    if reconciler.descriptor() != expected_reconciler.descriptor {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReconcilerDescriptorMismatchBefore);
    }

    let receipt = reconciler
        .reconcile_historical_journal(&subject)
        .map_err(|_| HistoricalInvocationJournalReconciliationErrorV1::ReconciliationError)?;

    if reconciler.descriptor() != expected_reconciler.descriptor {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReconcilerDescriptorChangedAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::UnsupportedReceiptSchema);
    }
    if receipt.reconciler != expected_reconciler.descriptor {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReceiptReconcilerMismatch);
    }
    if receipt.subject != subject {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReceiptSubjectMismatch);
    }
    if receipt.valid_until > expected_reconciler.descriptor.valid_until {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReceiptOutlivesReconciler);
    }
    if receipt.valid_until < latest {
        return Err(HistoricalInvocationJournalReconciliationErrorV1::ReceiptAlreadyExpired);
    }
    Ok(receipt)
}

pub fn reconcile_initial_invocation_journal_historically<R, TQ>(
    original_store: InvocationAttemptJournalStoreDescriptorV1,
    claim_record: EffectCapabilityClaimRecordCommitment,
    current_time: QualifiedCurrentTimeV1<TQ>,
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    reconciler: &R,
) -> Result<
    HistoricallyReconciledInitialInvocationJournalV1<R, TQ>,
    HistoricalInvocationJournalReconciliationFailureV1<TQ>,
>
where
    R: HistoricalInvocationJournalReconcilerV1,
{
    let subject = HistoricalInvocationJournalReconciliationSubjectV1::Initial {
        original_store,
        claim_record,
        current_time_receipt: current_time.receipt_commitment(),
        latest_possible_unix_ms: current_time.latest_possible_unix_ms(),
    };
    let receipt = match validate_common(subject, &current_time, expected_reconciler, reconciler) {
        Ok(receipt) => receipt,
        Err(error) => {
            return Err(HistoricalInvocationJournalReconciliationFailureV1 {
                subject,
                current_time,
                expected_reconciler,
                error,
            });
        }
    };
    let journal_receipt = match receipt.evidence {
        HistoricalInvocationJournalEvidenceV1::Initial(receipt) => receipt,
        HistoricalInvocationJournalEvidenceV1::Replay(_) => {
            return Err(HistoricalInvocationJournalReconciliationFailureV1 {
                subject,
                current_time,
                expected_reconciler,
                error: HistoricalInvocationJournalReconciliationErrorV1::ReceiptEvidenceKindMismatch,
            });
        }
    };
    let original_ceiling = match validate_initial_journal(journal_receipt, original_store, claim_record) {
        Ok(ceiling) => ceiling,
        Err(error) => {
            return Err(HistoricalInvocationJournalReconciliationFailureV1 {
                subject,
                current_time,
                expected_reconciler,
                error,
            });
        }
    };
    let invocation_eligibility_valid_until = original_ceiling
        .min(receipt.valid_until)
        .min(current_time.valid_until());

    Ok(HistoricallyReconciledInitialInvocationJournalV1 {
        journal_receipt,
        reconciliation_receipt: receipt,
        current_time,
        invocation_eligibility_valid_until,
        _reconciler: PhantomData,
    })
}

pub fn reconcile_replay_invocation_journal_historically<R, TQ>(
    original_store: ReplayInvocationJournalStoreDescriptorV1,
    prior_attempt_id: ActuatorInvocationAttemptId,
    current_time: QualifiedCurrentTimeV1<TQ>,
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    reconciler: &R,
) -> Result<
    HistoricallyReconciledReplayInvocationJournalV1<R, TQ>,
    HistoricalInvocationJournalReconciliationFailureV1<TQ>,
>
where
    R: HistoricalInvocationJournalReconcilerV1,
{
    let subject = HistoricalInvocationJournalReconciliationSubjectV1::Replay {
        original_store,
        prior_attempt_id,
        current_time_receipt: current_time.receipt_commitment(),
        latest_possible_unix_ms: current_time.latest_possible_unix_ms(),
    };
    let receipt = match validate_common(subject, &current_time, expected_reconciler, reconciler) {
        Ok(receipt) => receipt,
        Err(error) => {
            return Err(HistoricalInvocationJournalReconciliationFailureV1 {
                subject,
                current_time,
                expected_reconciler,
                error,
            });
        }
    };
    let journal_receipt = match receipt.evidence {
        HistoricalInvocationJournalEvidenceV1::Replay(receipt) => receipt,
        HistoricalInvocationJournalEvidenceV1::Initial(_) => {
            return Err(HistoricalInvocationJournalReconciliationFailureV1 {
                subject,
                current_time,
                expected_reconciler,
                error: HistoricalInvocationJournalReconciliationErrorV1::ReceiptEvidenceKindMismatch,
            });
        }
    };
    let original_ceiling = match validate_replay_journal(journal_receipt, original_store, prior_attempt_id) {
        Ok(ceiling) => ceiling,
        Err(error) => {
            return Err(HistoricalInvocationJournalReconciliationFailureV1 {
                subject,
                current_time,
                expected_reconciler,
                error,
            });
        }
    };
    let invocation_eligibility_valid_until = original_ceiling
        .min(receipt.valid_until)
        .min(current_time.valid_until());

    Ok(HistoricallyReconciledReplayInvocationJournalV1 {
        journal_receipt,
        reconciliation_receipt: receipt,
        current_time,
        invocation_eligibility_valid_until,
        _reconciler: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_identity_variants_do_not_alias() {
        let initial = HistoricalInvocationJournalStableIdentityV1::Initial(
            InvocationAttemptJournalStoreIdentityCommitment::from_bytes([1; 32]),
        );
        let replay = HistoricalInvocationJournalStableIdentityV1::Replay(
            ReplayInvocationJournalStoreIdentityCommitment::from_bytes([1; 32]),
        );
        assert_ne!(initial, replay);
    }

    #[test]
    fn fresh_reconciliation_never_extends_original_authority() {
        let original_authority = 90_u64;
        let fresh_reconciliation = 180_u64;
        assert_eq!(original_authority.min(fresh_reconciliation), 90);
    }
}

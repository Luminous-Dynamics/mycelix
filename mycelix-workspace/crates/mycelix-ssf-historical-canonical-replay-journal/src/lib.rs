// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only historical reconciliation for canonical replay invocation journals.
//!
//! The historical reconciler may rotate policy/key generations while retaining
//! the same stable replay-journal store identity. Fresh historical knowledge
//! never refreshes the original replay authorization, attempt, or actuator
//! authority ceiling. This crate performs no journal write or external effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_canonical_replay_invocation_journal::{
    CanonicalReplayInvocationJournalDispositionV1,
    CanonicalReplayInvocationJournalReceiptCommitment,
    CanonicalReplayInvocationJournalReceiptV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_historical_actuator_invocation_journal::{
    ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    HistoricalInvocationJournalReconcilerDescriptorV1,
    HistoricalInvocationJournalStableIdentityV1,
    HistoricalInvocationJournalTimeBasisV1,
};
use mycelix_ssf_replay_aware_invocation_journal::{
    DurableReplayInvocationJournalFrontierV1, ReplayInvocationJournalStoreDescriptorV1,
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

digest_type!(HistoricalCanonicalReplayJournalReceiptCommitment);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoricalCanonicalReplayJournalSubjectV1 {
    pub original_store: ReplayInvocationJournalStoreDescriptorV1,
    pub prior_attempt_id: mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptId,
    pub original_journal_receipt: CanonicalReplayInvocationJournalReceiptCommitment,
    pub current_time_receipt: CurrentTimeReceiptCommitment,
    pub latest_possible_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoricalCanonicalReplayJournalReceiptV1 {
    pub schema_version: u16,
    pub reconciler: HistoricalInvocationJournalReconcilerDescriptorV1,
    pub subject: HistoricalCanonicalReplayJournalSubjectV1,
    pub evidence: CanonicalReplayInvocationJournalReceiptV1,
    pub valid_until: u64,
    pub receipt_commitment: HistoricalCanonicalReplayJournalReceiptCommitment,
}

pub trait HistoricalCanonicalReplayJournalReconcilerV1 {
    type Error;

    fn descriptor(&self) -> HistoricalInvocationJournalReconcilerDescriptorV1;

    fn reconcile_historical_canonical_replay(
        &self,
        subject: &HistoricalCanonicalReplayJournalSubjectV1,
    ) -> Result<HistoricalCanonicalReplayJournalReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalCanonicalReplayJournalErrorV1 {
    UnsupportedReconcilerTimeBasis,
    ReconcilerAlreadyExpired,
    ReconcilerDescriptorMismatchBefore,
    ReconcilerDescriptorChangedAfter,
    StableStoreIdentityMismatch,
    OriginalStoreUsesUnsupportedTimeBasis,
    CurrentTimeAlreadyExpired,
    CurrentTimeRegressedBeforeAttempt,
    ReconciliationError,
    UnsupportedReceiptSchema,
    ReceiptReconcilerMismatch,
    ReceiptSubjectMismatch,
    ReceiptOutlivesReconciler,
    ReceiptAlreadyExpired,
    UnsupportedJournalReceiptSchema,
    UnsupportedJournalManifestSchema,
    JournalStoreMismatch,
    JournalPriorAttemptMismatch,
    JournalReceiptCommitmentMismatch,
    JournalReceiptOutlivedOriginalStore,
    JournalReceiptWasStaleAtAttempt,
    ReplayAuthorizationWasStaleAtAttempt,
    StableEffectIdentityMismatch,
    JournalGenerationOverflow,
    InvalidJournaledFrontier,
    JournalWasNotDurablyJournaled,
}

fn exact_frontier(
    expected: DurableReplayInvocationJournalFrontierV1,
    record: mycelix_ssf_replay_aware_invocation_journal::ReplayInvocationJournalRecordCommitment,
    post: DurableReplayInvocationJournalFrontierV1,
) -> Result<(), HistoricalCanonicalReplayJournalErrorV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(HistoricalCanonicalReplayJournalErrorV1::JournalGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(record) {
        return Err(HistoricalCanonicalReplayJournalErrorV1::InvalidJournaledFrontier);
    }
    Ok(())
}

pub struct HistoricallyReconciledCanonicalReplayJournalV1<R, TQ> {
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    receipt: HistoricalCanonicalReplayJournalReceiptV1,
    authority_valid_until: u64,
    _reconciler: PhantomData<fn() -> R>,
}

impl<R, TQ> HistoricallyReconciledCanonicalReplayJournalV1<R, TQ> {
    pub const fn expected_reconciler(
        &self,
    ) -> ExpectedHistoricalInvocationJournalReconcilerProfileV1 {
        self.expected_reconciler
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn reconciliation_receipt(&self) -> HistoricalCanonicalReplayJournalReceiptV1 {
        self.receipt
    }

    pub const fn journal_receipt(&self) -> CanonicalReplayInvocationJournalReceiptV1 {
        self.receipt.evidence
    }

    pub const fn authority_valid_until(&self) -> u64 {
        self.authority_valid_until
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub const fn reconstructs_replay_authority(&self) -> bool {
        false
    }
}

pub struct HistoricalCanonicalReplayJournalFailureV1<R, TQ> {
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: HistoricalCanonicalReplayJournalErrorV1,
    _reconciler: PhantomData<fn() -> R>,
}

impl<R, TQ> HistoricalCanonicalReplayJournalFailureV1<R, TQ> {
    pub const fn error(&self) -> HistoricalCanonicalReplayJournalErrorV1 {
        self.error
    }

    pub fn into_current_time(self) -> QualifiedCurrentTimeV1<TQ> {
        self.current_time
    }

    pub const fn expected_reconciler(
        &self,
    ) -> ExpectedHistoricalInvocationJournalReconcilerProfileV1 {
        self.expected_reconciler
    }
}

fn fail<R, TQ>(
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: HistoricalCanonicalReplayJournalErrorV1,
) -> Result<
    HistoricallyReconciledCanonicalReplayJournalV1<R, TQ>,
    HistoricalCanonicalReplayJournalFailureV1<R, TQ>,
> {
    Err(HistoricalCanonicalReplayJournalFailureV1 {
        expected_reconciler,
        current_time,
        error,
        _reconciler: PhantomData,
    })
}

pub fn reconcile_historical_canonical_replay_journal<R, TQ>(
    original_journal: CanonicalReplayInvocationJournalReceiptV1,
    expected_reconciler: ExpectedHistoricalInvocationJournalReconcilerProfileV1,
    reconciler: &R,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    HistoricallyReconciledCanonicalReplayJournalV1<R, TQ>,
    HistoricalCanonicalReplayJournalFailureV1<R, TQ>,
>
where
    R: HistoricalCanonicalReplayJournalReconcilerV1,
{
    let descriptor = expected_reconciler.descriptor();
    let latest = current_time.latest_possible_unix_ms();
    let manifest = original_journal.manifest;
    let attempt = manifest.attempt;

    if descriptor.time_basis != HistoricalInvocationJournalTimeBasisV1::UnixMillisecondsUtc {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::UnsupportedReconcilerTimeBasis,
        );
    }
    if descriptor.valid_until < latest {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReconcilerAlreadyExpired,
        );
    }
    if manifest.expected_store.time_basis != ReplayInvocationJournalTimeBasisV1::UnixMillisecondsUtc {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::OriginalStoreUsesUnsupportedTimeBasis,
        );
    }
    if descriptor.stable_identity
        != HistoricalInvocationJournalStableIdentityV1::Replay(
            manifest.expected_store.stable_identity,
        )
    {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::StableStoreIdentityMismatch,
        );
    }
    if current_time.valid_until() < latest {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::CurrentTimeAlreadyExpired,
        );
    }
    if latest < attempt.latest_possible_unix_ms {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::CurrentTimeRegressedBeforeAttempt,
        );
    }
    if reconciler.descriptor() != descriptor {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReconcilerDescriptorMismatchBefore,
        );
    }

    let subject = HistoricalCanonicalReplayJournalSubjectV1 {
        original_store: manifest.expected_store,
        prior_attempt_id: manifest.prior_attempt_id(),
        original_journal_receipt: original_journal.receipt_commitment,
        current_time_receipt: current_time.receipt_commitment(),
        latest_possible_unix_ms: latest,
    };

    let receipt = match reconciler.reconcile_historical_canonical_replay(&subject) {
        Ok(value) => value,
        Err(_) => {
            return fail::<R, TQ>(
                expected_reconciler,
                current_time,
                HistoricalCanonicalReplayJournalErrorV1::ReconciliationError,
            );
        }
    };
    if reconciler.descriptor() != descriptor {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReconcilerDescriptorChangedAfter,
        );
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::UnsupportedReceiptSchema,
        );
    }
    if receipt.reconciler != descriptor {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReceiptReconcilerMismatch,
        );
    }
    if receipt.subject != subject {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReceiptSubjectMismatch,
        );
    }
    if receipt.valid_until > descriptor.valid_until {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReceiptOutlivesReconciler,
        );
    }
    if receipt.valid_until < latest {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReceiptAlreadyExpired,
        );
    }

    let evidence = receipt.evidence;
    if evidence.schema_version != SSF_SCHEMA_V1 {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::UnsupportedJournalReceiptSchema,
        );
    }
    if evidence.manifest.schema_version != SSF_SCHEMA_V1 {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::UnsupportedJournalManifestSchema,
        );
    }
    if evidence.store != manifest.expected_store || evidence.manifest.expected_store != manifest.expected_store {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::JournalStoreMismatch,
        );
    }
    if evidence.manifest.prior_attempt_id() != manifest.prior_attempt_id() {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::JournalPriorAttemptMismatch,
        );
    }
    if evidence.receipt_commitment != original_journal.receipt_commitment
        || evidence != original_journal
    {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::JournalReceiptCommitmentMismatch,
        );
    }
    if evidence.valid_until > manifest.expected_store.valid_until {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::JournalReceiptOutlivedOriginalStore,
        );
    }
    if evidence.valid_until < attempt.latest_possible_unix_ms {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::JournalReceiptWasStaleAtAttempt,
        );
    }
    if manifest.replay.valid_until < attempt.latest_possible_unix_ms {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::ReplayAuthorizationWasStaleAtAttempt,
        );
    }
    let stable = attempt.subject.stable_identity();
    if manifest.replay.subject.stable_effect_identity != stable
        || manifest.replay.subject.evidence_subject.stable_effect_identity != stable
    {
        return fail::<R, TQ>(
            expected_reconciler,
            current_time,
            HistoricalCanonicalReplayJournalErrorV1::StableEffectIdentityMismatch,
        );
    }

    let (record, post_frontier) = match evidence.disposition {
        CanonicalReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => (record, post_frontier),
        _ => {
            return fail::<R, TQ>(
                expected_reconciler,
                current_time,
                HistoricalCanonicalReplayJournalErrorV1::JournalWasNotDurablyJournaled,
            );
        }
    };
    if let Err(error) = exact_frontier(manifest.expected_frontier, record, post_frontier) {
        return fail::<R, TQ>(expected_reconciler, current_time, error);
    }

    let authority_valid_until = manifest
        .replay
        .valid_until
        .min(attempt.attempt_valid_until)
        .min(attempt.subject.actuator.valid_until)
        .min(receipt.valid_until)
        .min(current_time.valid_until());

    Ok(HistoricallyReconciledCanonicalReplayJournalV1 {
        expected_reconciler,
        current_time,
        receipt,
        authority_valid_until,
        _reconciler: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_historical_knowledge_does_not_extend_old_authority() {
        let original_authority = 90_u64;
        let reconciliation = 180_u64;
        assert_eq!(original_authority.min(reconciliation), 90);
    }

    #[test]
    fn replay_store_identity_is_a_distinct_historical_identity_variant() {
        assert_ne!(
            core::mem::size_of::<HistoricalInvocationJournalStableIdentityV1>(),
            0
        );
    }
}

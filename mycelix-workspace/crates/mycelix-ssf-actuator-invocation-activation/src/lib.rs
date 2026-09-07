// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fresh activation of an already durably journaled SSF actuator invocation.
//!
//! Durable journaling proves that an invocation lineage exists. It does not
//! prove that the lineage is still live when external effect work finally
//! begins. This crate adds a shared fresh-time activation boundary for both the
//! initial journal lane and the explicitly authorized replay lane, including
//! exact read-only recovery evidence after process restart.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptManifestV1;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_durable_actuator_invocation_attempt::{
    DurableInvocationAttemptJournalFrontierV1, DurablyJournaledActuatorEffectAttemptV1,
    InvocationAttemptJournalDispositionV1, InvocationAttemptJournalManifestV1,
    InvocationAttemptJournalReceiptCommitment, InvocationAttemptJournalRecordCommitment,
    InvocationAttemptJournalTimeBasisV1, RecoveredInvocationAttemptJournalV1,
};
use mycelix_ssf_replay_aware_invocation_journal::{
    DurableReplayInvocationJournalFrontierV1, DurablyJournaledReplayAttemptV1,
    RecoveredReplayInvocationJournalV1, ReplayInvocationJournalDispositionV1,
    ReplayInvocationJournalManifestV1, ReplayInvocationJournalReceiptCommitment,
    ReplayInvocationJournalRecordCommitment, ReplayInvocationJournalTimeBasisV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvocationJournalEvidenceV1 {
    Initial {
        record: InvocationAttemptJournalRecordCommitment,
        receipt: InvocationAttemptJournalReceiptCommitment,
        valid_until: u64,
    },
    Replay {
        record: ReplayInvocationJournalRecordCommitment,
        receipt: ReplayInvocationJournalReceiptCommitment,
        valid_until: u64,
    },
}

impl InvocationJournalEvidenceV1 {
    pub const fn valid_until(self) -> u64 {
        match self {
            Self::Initial { valid_until, .. } | Self::Replay { valid_until, .. } => valid_until,
        }
    }

    pub const fn is_replay(self) -> bool {
        matches!(self, Self::Replay { .. })
    }
}

fn initial_frontier_is_exact(
    manifest: InvocationAttemptJournalManifestV1,
    record: InvocationAttemptJournalRecordCommitment,
    post: DurableInvocationAttemptJournalFrontierV1,
) -> bool {
    manifest
        .expected_frontier
        .generation
        .get()
        .checked_add(1)
        .is_some_and(|next| post.generation.get() == next && post.head == Some(record))
}

fn replay_frontier_is_exact(
    manifest: ReplayInvocationJournalManifestV1,
    record: ReplayInvocationJournalRecordCommitment,
    post: DurableReplayInvocationJournalFrontierV1,
) -> bool {
    manifest
        .expected_frontier
        .generation
        .get()
        .checked_add(1)
        .is_some_and(|next| post.generation.get() == next && post.head == Some(record))
}

fn initial_receipt_binding_is_consistent(
    manifest: InvocationAttemptJournalManifestV1,
    receipt: mycelix_ssf_durable_actuator_invocation_attempt::InvocationAttemptJournalReceiptV1,
) -> bool {
    if manifest.schema_version != SSF_SCHEMA_V1
        || manifest.attempt.schema_version != SSF_SCHEMA_V1
        || receipt.schema_version != SSF_SCHEMA_V1
        || receipt.manifest != manifest
        || receipt.store != manifest.expected_store
        || receipt.valid_until > manifest.expected_store.valid_until
    {
        return false;
    }
    match receipt.disposition {
        InvocationAttemptJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => initial_frontier_is_exact(manifest, record, post_frontier),
        _ => false,
    }
}

fn replay_receipt_binding_is_consistent(
    manifest: ReplayInvocationJournalManifestV1,
    receipt: mycelix_ssf_replay_aware_invocation_journal::ReplayInvocationJournalReceiptV1,
) -> bool {
    if manifest.schema_version != SSF_SCHEMA_V1
        || manifest.attempt.schema_version != SSF_SCHEMA_V1
        || receipt.schema_version != SSF_SCHEMA_V1
        || receipt.manifest != manifest
        || receipt.store != manifest.expected_store
        || receipt.valid_until > manifest.expected_store.valid_until
        || manifest.replay.subject.prior_effect_subject.stable_identity()
            != manifest.replay.subject.stable_effect_identity
        || manifest.attempt.subject.stable_identity() != manifest.replay.subject.stable_effect_identity
    {
        return false;
    }
    match receipt.disposition {
        ReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => replay_frontier_is_exact(manifest, record, post_frontier),
        _ => false,
    }
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactDurablyJournaledInvocationV1: sealed::Sealed {
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1;
    fn exact_journal_evidence(&self) -> Option<InvocationJournalEvidenceV1>;
    fn exact_journal_binding_is_consistent(&self) -> bool;
    fn journal_uses_unix_milliseconds(&self) -> bool;
}

impl<S, P> sealed::Sealed for DurablyJournaledActuatorEffectAttemptV1<S, P> {}

impl<S, P> ExactDurablyJournaledInvocationV1 for DurablyJournaledActuatorEffectAttemptV1<S, P> {
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.manifest().attempt
    }

    fn exact_journal_evidence(&self) -> Option<InvocationJournalEvidenceV1> {
        let manifest = self.manifest();
        let receipt = self.receipt();
        match receipt.disposition {
            InvocationAttemptJournalDispositionV1::Journaled { record, .. } => {
                Some(InvocationJournalEvidenceV1::Initial {
                    record,
                    receipt: receipt.receipt_commitment,
                    valid_until: receipt.valid_until.min(manifest.expected_store.valid_until),
                })
            }
            _ => None,
        }
    }

    fn exact_journal_binding_is_consistent(&self) -> bool {
        initial_receipt_binding_is_consistent(self.manifest(), self.receipt())
            && matches!(
                self.receipt().disposition,
                InvocationAttemptJournalDispositionV1::Journaled {
                    record,
                    post_frontier,
                } if record == self.record() && post_frontier == self.post_frontier()
            )
    }

    fn journal_uses_unix_milliseconds(&self) -> bool {
        self.manifest().expected_store.time_basis
            == InvocationAttemptJournalTimeBasisV1::UnixMillisecondsUtc
    }
}

impl<S> sealed::Sealed for RecoveredInvocationAttemptJournalV1<S> {}

impl<S> ExactDurablyJournaledInvocationV1 for RecoveredInvocationAttemptJournalV1<S> {
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.receipt().manifest.attempt
    }

    fn exact_journal_evidence(&self) -> Option<InvocationJournalEvidenceV1> {
        let receipt = self.receipt();
        let manifest = receipt.manifest;
        match receipt.disposition {
            InvocationAttemptJournalDispositionV1::Journaled { record, .. } => {
                Some(InvocationJournalEvidenceV1::Initial {
                    record,
                    receipt: receipt.receipt_commitment,
                    valid_until: receipt.valid_until.min(manifest.expected_store.valid_until),
                })
            }
            _ => None,
        }
    }

    fn exact_journal_binding_is_consistent(&self) -> bool {
        let receipt = self.receipt();
        initial_receipt_binding_is_consistent(receipt.manifest, receipt)
    }

    fn journal_uses_unix_milliseconds(&self) -> bool {
        self.receipt().manifest.expected_store.time_basis
            == InvocationAttemptJournalTimeBasisV1::UnixMillisecondsUtc
    }
}

impl<R, P, S, TQ, Q, QTQ, ITQ> sealed::Sealed
    for DurablyJournaledReplayAttemptV1<R, P, S, TQ, Q, QTQ, ITQ>
{
}

impl<R, P, S, TQ, Q, QTQ, ITQ> ExactDurablyJournaledInvocationV1
    for DurablyJournaledReplayAttemptV1<R, P, S, TQ, Q, QTQ, ITQ>
{
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.manifest().attempt
    }

    fn exact_journal_evidence(&self) -> Option<InvocationJournalEvidenceV1> {
        let manifest = self.manifest();
        let receipt = self.receipt();
        match receipt.disposition {
            ReplayInvocationJournalDispositionV1::Journaled { record, .. } => {
                Some(InvocationJournalEvidenceV1::Replay {
                    record,
                    receipt: receipt.receipt_commitment,
                    valid_until: receipt
                        .valid_until
                        .min(manifest.expected_store.valid_until)
                        .min(manifest.replay.valid_until),
                })
            }
            _ => None,
        }
    }

    fn exact_journal_binding_is_consistent(&self) -> bool {
        replay_receipt_binding_is_consistent(self.manifest(), self.receipt())
            && matches!(
                self.receipt().disposition,
                ReplayInvocationJournalDispositionV1::Journaled {
                    record,
                    post_frontier,
                } if record == self.record() && post_frontier == self.post_frontier()
            )
    }

    fn journal_uses_unix_milliseconds(&self) -> bool {
        self.manifest().expected_store.time_basis
            == ReplayInvocationJournalTimeBasisV1::UnixMillisecondsUtc
    }
}

impl<R> sealed::Sealed for RecoveredReplayInvocationJournalV1<R> {}

impl<R> ExactDurablyJournaledInvocationV1 for RecoveredReplayInvocationJournalV1<R> {
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.receipt().manifest.attempt
    }

    fn exact_journal_evidence(&self) -> Option<InvocationJournalEvidenceV1> {
        let receipt = self.receipt();
        let manifest = receipt.manifest;
        match receipt.disposition {
            ReplayInvocationJournalDispositionV1::Journaled { record, .. } => {
                Some(InvocationJournalEvidenceV1::Replay {
                    record,
                    receipt: receipt.receipt_commitment,
                    valid_until: receipt
                        .valid_until
                        .min(manifest.expected_store.valid_until)
                        .min(manifest.replay.valid_until),
                })
            }
            _ => None,
        }
    }

    fn exact_journal_binding_is_consistent(&self) -> bool {
        let receipt = self.receipt();
        replay_receipt_binding_is_consistent(receipt.manifest, receipt)
    }

    fn journal_uses_unix_milliseconds(&self) -> bool {
        self.receipt().manifest.expected_store.time_basis
            == ReplayInvocationJournalTimeBasisV1::UnixMillisecondsUtc
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvocationActivationErrorV1 {
    UnsupportedAttemptSchema,
    JournalBindingMismatch,
    UnsupportedJournalTimeBasis,
    CurrentTimeAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
    JournalEvidenceAlreadyExpired,
}

pub struct InvocationActivationFailureV1<J, TQ> {
    journaled: J,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: InvocationActivationErrorV1,
}

impl<J, TQ> InvocationActivationFailureV1<J, TQ> {
    pub const fn error(&self) -> InvocationActivationErrorV1 {
        self.error
    }

    pub fn into_parts(self) -> (J, QualifiedCurrentTimeV1<TQ>) {
        (self.journaled, self.current_time)
    }
}

pub struct ActivatedActuatorInvocationV1<J, TQ> {
    journaled: J,
    current_time: QualifiedCurrentTimeV1<TQ>,
    attempt: ActuatorInvocationAttemptManifestV1,
    journal_evidence: InvocationJournalEvidenceV1,
    valid_until: u64,
}

impl<J, TQ> ActivatedActuatorInvocationV1<J, TQ> {
    pub const fn journaled(&self) -> &J {
        &self.journaled
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn journal_evidence(&self) -> InvocationJournalEvidenceV1 {
        self.journal_evidence
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn eligible_for_actuator_reservation(&self) -> bool {
        true
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }
}

pub fn activate_durably_journaled_invocation<J, TQ>(
    journaled: J,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<ActivatedActuatorInvocationV1<J, TQ>, InvocationActivationFailureV1<J, TQ>>
where
    J: ExactDurablyJournaledInvocationV1,
{
    let attempt = journaled.exact_attempt_manifest();
    let latest = current_time.latest_possible_unix_ms();

    if attempt.schema_version != SSF_SCHEMA_V1 {
        return Err(InvocationActivationFailureV1 {
            journaled,
            current_time,
            error: InvocationActivationErrorV1::UnsupportedAttemptSchema,
        });
    }
    if !journaled.exact_journal_binding_is_consistent() {
        return Err(InvocationActivationFailureV1 {
            journaled,
            current_time,
            error: InvocationActivationErrorV1::JournalBindingMismatch,
        });
    }
    let journal_evidence = match journaled.exact_journal_evidence() {
        Some(evidence) => evidence,
        None => {
            return Err(InvocationActivationFailureV1 {
                journaled,
                current_time,
                error: InvocationActivationErrorV1::JournalBindingMismatch,
            });
        }
    };

    let error = if !journaled.journal_uses_unix_milliseconds() {
        Some(InvocationActivationErrorV1::UnsupportedJournalTimeBasis)
    } else if current_time.valid_until() < latest {
        Some(InvocationActivationErrorV1::CurrentTimeAlreadyExpired)
    } else if attempt.attempt_valid_until < latest {
        Some(InvocationActivationErrorV1::AttemptAlreadyExpired)
    } else if attempt.subject.actuator.valid_until < latest {
        Some(InvocationActivationErrorV1::ActuatorGenerationAlreadyExpired)
    } else if journal_evidence.valid_until() < latest {
        Some(InvocationActivationErrorV1::JournalEvidenceAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(InvocationActivationFailureV1 {
            journaled,
            current_time,
            error,
        });
    }

    let valid_until = current_time
        .valid_until()
        .min(attempt.attempt_valid_until)
        .min(attempt.subject.actuator.valid_until)
        .min(journal_evidence.valid_until());

    Ok(ActivatedActuatorInvocationV1 {
        journaled,
        current_time,
        attempt,
        journal_evidence,
        valid_until,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evidence_kind_distinguishes_initial_and_replay() {
        let initial = InvocationJournalEvidenceV1::Initial {
            record: InvocationAttemptJournalRecordCommitment::from_bytes([1; 32]),
            receipt: InvocationAttemptJournalReceiptCommitment::from_bytes([2; 32]),
            valid_until: 10,
        };
        let replay = InvocationJournalEvidenceV1::Replay {
            record: ReplayInvocationJournalRecordCommitment::from_bytes([3; 32]),
            receipt: ReplayInvocationJournalReceiptCommitment::from_bytes([4; 32]),
            valid_until: 9,
        };
        assert!(!initial.is_replay());
        assert!(replay.is_replay());
        assert_eq!(initial.valid_until(), 10);
        assert_eq!(replay.valid_until(), 9);
    }

    #[test]
    fn activation_ceiling_can_only_shrink() {
        let ceiling = 100_u64.min(90).min(80).min(70);
        assert_eq!(ceiling, 70);
    }
}

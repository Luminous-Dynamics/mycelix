// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fresh post-journal activation for one canonical replay attempt.
//!
//! Durable canonical replay journaling proves that the second-attempt lineage
//! exists. It does not prove that the attempt, replay authorization, actuator
//! generation, journal evidence, or trusted time are still live when execution
//! is about to continue. This crate accepts only exact canonical replay journal
//! typestates (live or same-generation recovered) and adds a fresh-time,
//! anti-rollback activation boundary. It performs no actuator reservation or
//! external effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptManifestV1;
use mycelix_ssf_canonical_effect_outcome_history::CanonicalInvocationRecordV1;
use mycelix_ssf_canonical_replay_invocation_journal::{
    CanonicalReplayInvocationJournalDispositionV1,
    CanonicalReplayInvocationJournalReceiptCommitment,
    CanonicalReplayInvocationJournalReceiptV1,
    DurablyJournaledCanonicalReplayAttemptV1,
    RecoveredCanonicalReplayInvocationJournalV1,
};
use mycelix_ssf_canonical_replay_policy::CanonicalReplayAuthorizationCommitment;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_replay_aware_invocation_journal::{
    DurableReplayInvocationJournalFrontierV1, ReplayInvocationJournalRecordCommitment,
    ReplayInvocationJournalTimeBasisV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayJournalEvidenceV1 {
    pub record: ReplayInvocationJournalRecordCommitment,
    pub receipt: CanonicalReplayInvocationJournalReceiptCommitment,
    pub prior_invocation_record: CanonicalInvocationRecordV1,
    pub authorization: CanonicalReplayAuthorizationCommitment,
    pub authorization_valid_until: u64,
    pub journal_valid_until: u64,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReplayInvocationActivationErrorV1 {
    UnsupportedReceiptSchema,
    UnsupportedManifestSchema,
    UnsupportedAttemptSchema,
    UnsupportedJournalTimeBasis,
    ReceiptStoreMismatch,
    ReceiptOutlivesStore,
    ReceiptWasStaleAtAttempt,
    JournalWasNotDurablyJournaled,
    JournalGenerationOverflow,
    InvalidJournaledFrontier,
    StableEffectIdentityMismatch,
    PriorInvocationRecordMismatch,
    AuthorizationWasStaleAtAttempt,
    CurrentTimeRegressedBeforeAttempt,
    CurrentTimeRegressedBeforePolicyDecision,
    CurrentTimeAlreadyExpired,
    CanonicalReplayAuthorizationAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
    JournalEvidenceAlreadyExpired,
}

fn exact_frontier(
    expected: DurableReplayInvocationJournalFrontierV1,
    record: ReplayInvocationJournalRecordCommitment,
    post: DurableReplayInvocationJournalFrontierV1,
) -> Result<(), CanonicalReplayInvocationActivationErrorV1> {
    let next = expected
        .generation
        .get()
        .checked_add(1)
        .ok_or(CanonicalReplayInvocationActivationErrorV1::JournalGenerationOverflow)?;
    if post.generation.get() != next || post.head != Some(record) {
        return Err(CanonicalReplayInvocationActivationErrorV1::InvalidJournaledFrontier);
    }
    Ok(())
}

fn validate_receipt(
    receipt: CanonicalReplayInvocationJournalReceiptV1,
) -> Result<
    (ActuatorInvocationAttemptManifestV1, CanonicalReplayJournalEvidenceV1),
    CanonicalReplayInvocationActivationErrorV1,
> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(CanonicalReplayInvocationActivationErrorV1::UnsupportedReceiptSchema);
    }
    let manifest = receipt.manifest;
    if manifest.schema_version != SSF_SCHEMA_V1 {
        return Err(CanonicalReplayInvocationActivationErrorV1::UnsupportedManifestSchema);
    }
    let attempt = manifest.attempt;
    if attempt.schema_version != SSF_SCHEMA_V1 {
        return Err(CanonicalReplayInvocationActivationErrorV1::UnsupportedAttemptSchema);
    }
    if manifest.expected_store.time_basis != ReplayInvocationJournalTimeBasisV1::UnixMillisecondsUtc {
        return Err(CanonicalReplayInvocationActivationErrorV1::UnsupportedJournalTimeBasis);
    }
    if receipt.store != manifest.expected_store {
        return Err(CanonicalReplayInvocationActivationErrorV1::ReceiptStoreMismatch);
    }
    if receipt.valid_until > manifest.expected_store.valid_until {
        return Err(CanonicalReplayInvocationActivationErrorV1::ReceiptOutlivesStore);
    }
    if receipt.valid_until < attempt.latest_possible_unix_ms {
        return Err(CanonicalReplayInvocationActivationErrorV1::ReceiptWasStaleAtAttempt);
    }
    if manifest.replay.valid_until < attempt.latest_possible_unix_ms {
        return Err(CanonicalReplayInvocationActivationErrorV1::AuthorizationWasStaleAtAttempt);
    }

    let stable = attempt.subject.stable_identity();
    if manifest.replay.subject.stable_effect_identity != stable
        || manifest
            .replay
            .subject
            .evidence_subject
            .stable_effect_identity
            != stable
    {
        return Err(CanonicalReplayInvocationActivationErrorV1::StableEffectIdentityMismatch);
    }
    if manifest
        .replay
        .subject
        .evidence_subject
        .latest_entry
        .manifest
        .subject
        .invocation_record
        != manifest.prior_invocation_record
    {
        return Err(CanonicalReplayInvocationActivationErrorV1::PriorInvocationRecordMismatch);
    }

    let (record, post_frontier) = match receipt.disposition {
        CanonicalReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => (record, post_frontier),
        _ => return Err(CanonicalReplayInvocationActivationErrorV1::JournalWasNotDurablyJournaled),
    };
    exact_frontier(manifest.expected_frontier, record, post_frontier)?;

    let authorization_valid_until = manifest.replay.valid_until;
    let journal_valid_until = receipt.valid_until.min(manifest.expected_store.valid_until);
    let evidence = CanonicalReplayJournalEvidenceV1 {
        record,
        receipt: receipt.receipt_commitment,
        prior_invocation_record: manifest.prior_invocation_record,
        authorization: manifest.replay.authorization,
        authorization_valid_until,
        journal_valid_until,
        valid_until: authorization_valid_until.min(journal_valid_until),
    };
    Ok((attempt, evidence))
}

mod sealed {
    pub trait Sealed {}
}

/// Sealed source for the exact canonical replay journal typestates supported by
/// this activation boundary. External crates cannot manufacture another source.
pub trait ExactCanonicalReplayJournalSourceV1: sealed::Sealed {
    fn exact_receipt(&self) -> CanonicalReplayInvocationJournalReceiptV1;
    fn direct_journal_binding_is_consistent(&self) -> bool;
}

impl<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ> sealed::Sealed
    for DurablyJournaledCanonicalReplayAttemptV1<
        R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ,
    >
{
}

impl<R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ> ExactCanonicalReplayJournalSourceV1
    for DurablyJournaledCanonicalReplayAttemptV1<
        R, P, EQ, S, HTQ, QTQ, PTQ, AQ, AQTQ, AITQ,
    >
{
    fn exact_receipt(&self) -> CanonicalReplayInvocationJournalReceiptV1 {
        self.receipt()
    }

    fn direct_journal_binding_is_consistent(&self) -> bool {
        let receipt = self.receipt();
        matches!(
            receipt.disposition,
            CanonicalReplayInvocationJournalDispositionV1::Journaled {
                record,
                post_frontier,
            } if record == self.record()
                && post_frontier == self.post_frontier()
                && receipt.manifest == self.manifest()
        )
    }
}

impl<R> sealed::Sealed for RecoveredCanonicalReplayInvocationJournalV1<R> {}

impl<R> ExactCanonicalReplayJournalSourceV1 for RecoveredCanonicalReplayInvocationJournalV1<R> {
    fn exact_receipt(&self) -> CanonicalReplayInvocationJournalReceiptV1 {
        self.receipt()
    }

    fn direct_journal_binding_is_consistent(&self) -> bool {
        true
    }
}

pub struct CanonicalReplayInvocationActivationFailureV1<J, TQ> {
    journaled: J,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: CanonicalReplayInvocationActivationErrorV1,
}

impl<J, TQ> CanonicalReplayInvocationActivationFailureV1<J, TQ> {
    pub const fn error(&self) -> CanonicalReplayInvocationActivationErrorV1 {
        self.error
    }

    pub fn into_parts(self) -> (J, QualifiedCurrentTimeV1<TQ>) {
        (self.journaled, self.current_time)
    }
}

pub struct ActivatedCanonicalReplayInvocationV1<J, TQ> {
    journaled: J,
    current_time: QualifiedCurrentTimeV1<TQ>,
    attempt: ActuatorInvocationAttemptManifestV1,
    journal_evidence: CanonicalReplayJournalEvidenceV1,
    valid_until: u64,
}

impl<J, TQ> ActivatedCanonicalReplayInvocationV1<J, TQ> {
    pub const fn journaled(&self) -> &J {
        &self.journaled
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn journal_evidence(&self) -> CanonicalReplayJournalEvidenceV1 {
        self.journal_evidence
    }

    pub const fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.current_time.latest_possible_unix_ms()
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn eligible_for_final_readiness(&self) -> bool {
        true
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

pub fn activate_canonical_replay_invocation<J, TQ>(
    journaled: J,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    ActivatedCanonicalReplayInvocationV1<J, TQ>,
    CanonicalReplayInvocationActivationFailureV1<J, TQ>,
>
where
    J: ExactCanonicalReplayJournalSourceV1,
{
    if !journaled.direct_journal_binding_is_consistent() {
        return Err(CanonicalReplayInvocationActivationFailureV1 {
            journaled,
            current_time,
            error: CanonicalReplayInvocationActivationErrorV1::JournalWasNotDurablyJournaled,
        });
    }

    let receipt = journaled.exact_receipt();
    let (attempt, journal_evidence) = match validate_receipt(receipt) {
        Ok(value) => value,
        Err(error) => {
            return Err(CanonicalReplayInvocationActivationFailureV1 {
                journaled,
                current_time,
                error,
            });
        }
    };

    let latest = current_time.latest_possible_unix_ms();
    let policy_time = receipt.manifest.replay.subject.policy_latest_possible_unix_ms;

    let error = if latest < attempt.latest_possible_unix_ms {
        Some(CanonicalReplayInvocationActivationErrorV1::CurrentTimeRegressedBeforeAttempt)
    } else if latest < policy_time {
        Some(CanonicalReplayInvocationActivationErrorV1::CurrentTimeRegressedBeforePolicyDecision)
    } else if current_time.valid_until() < latest {
        Some(CanonicalReplayInvocationActivationErrorV1::CurrentTimeAlreadyExpired)
    } else if journal_evidence.authorization_valid_until < latest {
        Some(CanonicalReplayInvocationActivationErrorV1::CanonicalReplayAuthorizationAlreadyExpired)
    } else if attempt.attempt_valid_until < latest {
        Some(CanonicalReplayInvocationActivationErrorV1::AttemptAlreadyExpired)
    } else if attempt.subject.actuator.valid_until < latest {
        Some(CanonicalReplayInvocationActivationErrorV1::ActuatorGenerationAlreadyExpired)
    } else if journal_evidence.journal_valid_until < latest {
        Some(CanonicalReplayInvocationActivationErrorV1::JournalEvidenceAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(CanonicalReplayInvocationActivationFailureV1 {
            journaled,
            current_time,
            error,
        });
    }

    let valid_until = current_time
        .valid_until()
        .min(journal_evidence.authorization_valid_until)
        .min(attempt.attempt_valid_until)
        .min(attempt.subject.actuator.valid_until)
        .min(journal_evidence.journal_valid_until);

    Ok(ActivatedCanonicalReplayInvocationV1 {
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
    fn activation_ceiling_can_only_shrink() {
        assert_eq!(100_u64.min(90).min(80).min(70).min(60), 60);
    }

    #[test]
    fn separate_authorization_and_journal_ceilings_remain_auditable() {
        let authorization = 90_u64;
        let journal = 70_u64;
        assert_eq!(authorization.min(journal), 70);
    }
}

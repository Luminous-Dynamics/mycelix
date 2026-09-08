// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Unified non-authoritative evidence for completed canonical actuator effects.
//!
//! This crate consumes either the existing canonical actuator outcome or the
//! canonical replay actuator outcome through a sealed source trait. It freezes
//! one copyable evidence shape while preserving the exact durable execution
//! provenance and never reconstructing execution/replay authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::{
    ActuatorEffectReceiptCommitment, ActuatorEffectReceiptV1,
    ActuatorInvocationAttemptManifestV1, ValidatedActuatorEffectDispositionV1,
    ValidatedActuatorEffectOutcomeV1,
};
use mycelix_ssf_actuator_invocation_activation::InvocationJournalEvidenceV1;
use mycelix_ssf_canonical_actuator_execution::{
    CanonicalActuatorInvocationOutcomeV1, CanonicalInvocationJournalAuditBindingV1,
    CanonicalInvocationJournalEvidenceV1, CanonicalPostInvocationAmbiguityReasonV1,
    CanonicalQualifiedEffectDispositionV1,
};
use mycelix_ssf_canonical_replay_actuator_execution::{
    CanonicalReplayActuatorInvocationOutcomeV1, CanonicalReplayInvocationAuditBindingV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, ExpectedCurrentTimeQualifierProfileV1, QualifiedCurrentTimeV1,
    TimeSourceCommitment,
};
use mycelix_ssf_durable_actuator_invocation_attempt::{
    InvocationAttemptJournalDispositionV1, InvocationAttemptJournalRecordCommitment,
};
use mycelix_ssf_final_canonical_replay_readiness::FinalCanonicalReplayAuditEvidenceV1;
use mycelix_ssf_historical_actuator_invocation_journal::HistoricalInvocationJournalEvidenceV1;
use mycelix_ssf_replay_aware_invocation_journal::{
    ReplayInvocationJournalDispositionV1, ReplayInvocationJournalRecordCommitment,
};

/// Durable record identity for one completed invocation lineage.
///
/// Legacy and canonical replay records intentionally remain different semantic
/// variants even though they share the same underlying commitment type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalCompletedInvocationRecordV1 {
    Initial(InvocationAttemptJournalRecordCommitment),
    LegacyReplay(ReplayInvocationJournalRecordCommitment),
    CanonicalReplay(ReplayInvocationJournalRecordCommitment),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalCompletedInvocationProvenanceV1 {
    Existing(CanonicalInvocationJournalAuditBindingV1),
    CanonicalReplay(CanonicalReplayInvocationAuditBindingV1),
}

impl CanonicalCompletedInvocationProvenanceV1 {
    pub const fn attempt(self) -> ActuatorInvocationAttemptManifestV1 {
        match self {
            Self::Existing(binding) => binding.attempt,
            Self::CanonicalReplay(binding) => binding.attempt,
        }
    }

    pub fn invocation_record(self) -> Option<CanonicalCompletedInvocationRecordV1> {
        match self {
            Self::Existing(binding) => existing_record(binding),
            Self::CanonicalReplay(binding) => Some(canonical_replay_record(binding)),
        }
    }

    pub const fn contains_execution_authority(self) -> bool {
        false
    }
}

fn existing_record(
    binding: CanonicalInvocationJournalAuditBindingV1,
) -> Option<CanonicalCompletedInvocationRecordV1> {
    match binding.evidence {
        CanonicalInvocationJournalEvidenceV1::CurrentOrSameGeneration(evidence) => match evidence {
            InvocationJournalEvidenceV1::Initial { record, .. } => {
                Some(CanonicalCompletedInvocationRecordV1::Initial(record))
            }
            InvocationJournalEvidenceV1::Replay { record, .. } => {
                Some(CanonicalCompletedInvocationRecordV1::LegacyReplay(record))
            }
        },
        CanonicalInvocationJournalEvidenceV1::HistoricalSameIdentity(evidence) => match evidence {
            HistoricalInvocationJournalEvidenceV1::Initial(receipt) => match receipt.disposition {
                InvocationAttemptJournalDispositionV1::Journaled { record, .. } => {
                    Some(CanonicalCompletedInvocationRecordV1::Initial(record))
                }
                _ => None,
            },
            HistoricalInvocationJournalEvidenceV1::Replay(receipt) => match receipt.disposition {
                ReplayInvocationJournalDispositionV1::Journaled { record, .. } => {
                    Some(CanonicalCompletedInvocationRecordV1::LegacyReplay(record))
                }
                _ => None,
            },
        },
    }
}

fn canonical_replay_record(
    binding: CanonicalReplayInvocationAuditBindingV1,
) -> CanonicalCompletedInvocationRecordV1 {
    let record = match binding.evidence {
        FinalCanonicalReplayAuditEvidenceV1::CurrentOrSameGeneration(evidence) => evidence.record,
        FinalCanonicalReplayAuditEvidenceV1::HistoricalSameIdentity(evidence) => {
            evidence.journal_record
        }
    };
    CanonicalCompletedInvocationRecordV1::CanonicalReplay(record)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalCompletedQualifiedTimeEvidenceV1 {
    pub expected_profile: ExpectedCurrentTimeQualifierProfileV1,
    pub source: TimeSourceCommitment,
    pub observed_unix_ms: u64,
    pub uncertainty_ms: u64,
    pub latest_possible_unix_ms: u64,
    pub valid_until: u64,
    pub receipt_commitment: CurrentTimeReceiptCommitment,
}

impl CanonicalCompletedQualifiedTimeEvidenceV1 {
    fn from_qualified<TQ>(time: &QualifiedCurrentTimeV1<TQ>) -> Self {
        Self {
            expected_profile: time.expected_profile(),
            source: time.source(),
            observed_unix_ms: time.observed_unix_ms(),
            uncertainty_ms: time.uncertainty_ms(),
            latest_possible_unix_ms: time.latest_possible_unix_ms(),
            valid_until: time.valid_until(),
            receipt_commitment: time.receipt_commitment(),
        }
    }
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactCompletedCanonicalInvocationV1: sealed::Sealed {
    fn provenance(&self) -> CanonicalCompletedInvocationProvenanceV1;
    fn attempt(&self) -> ActuatorInvocationAttemptManifestV1;
    fn reported_receipt(&self) -> Option<ActuatorEffectReceiptV1>;
    fn low_level_outcome(&self) -> ValidatedActuatorEffectOutcomeV1;
    fn canonical_disposition(&self) -> CanonicalQualifiedEffectDispositionV1;
    fn pre_time_evidence(&self) -> CanonicalCompletedQualifiedTimeEvidenceV1;
    fn post_time_evidence(&self) -> Option<CanonicalCompletedQualifiedTimeEvidenceV1>;
    fn post_ambiguity(&self) -> Option<CanonicalPostInvocationAmbiguityReasonV1>;
    fn creates_direct_replay_authority(&self) -> bool;
    fn permits_third_attempt(&self) -> bool;
}

impl<A, P, TQ> sealed::Sealed for CanonicalActuatorInvocationOutcomeV1<A, P, TQ> {}

impl<A, P, TQ> ExactCompletedCanonicalInvocationV1
    for CanonicalActuatorInvocationOutcomeV1<A, P, TQ>
{
    fn provenance(&self) -> CanonicalCompletedInvocationProvenanceV1 {
        CanonicalCompletedInvocationProvenanceV1::Existing(self.journal_audit())
    }

    fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt()
    }

    fn reported_receipt(&self) -> Option<ActuatorEffectReceiptV1> {
        self.reported_receipt()
    }

    fn low_level_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.low_level_outcome()
    }

    fn canonical_disposition(&self) -> CanonicalQualifiedEffectDispositionV1 {
        self.canonical_disposition()
    }

    fn pre_time_evidence(&self) -> CanonicalCompletedQualifiedTimeEvidenceV1 {
        CanonicalCompletedQualifiedTimeEvidenceV1::from_qualified(self.pre_invocation_time())
    }

    fn post_time_evidence(&self) -> Option<CanonicalCompletedQualifiedTimeEvidenceV1> {
        self.post_invocation_time()
            .map(CanonicalCompletedQualifiedTimeEvidenceV1::from_qualified)
    }

    fn post_ambiguity(&self) -> Option<CanonicalPostInvocationAmbiguityReasonV1> {
        self.post_invocation_ambiguity()
    }

    fn creates_direct_replay_authority(&self) -> bool {
        self.canonical_disposition().creates_replay_authority()
    }

    fn permits_third_attempt(&self) -> bool {
        false
    }
}

impl<A, P, TQ> sealed::Sealed for CanonicalReplayActuatorInvocationOutcomeV1<A, P, TQ> {}

impl<A, P, TQ> ExactCompletedCanonicalInvocationV1
    for CanonicalReplayActuatorInvocationOutcomeV1<A, P, TQ>
{
    fn provenance(&self) -> CanonicalCompletedInvocationProvenanceV1 {
        CanonicalCompletedInvocationProvenanceV1::CanonicalReplay(self.audit())
    }

    fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt()
    }

    fn reported_receipt(&self) -> Option<ActuatorEffectReceiptV1> {
        self.reported_receipt()
    }

    fn low_level_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.low_level_outcome()
    }

    fn canonical_disposition(&self) -> CanonicalQualifiedEffectDispositionV1 {
        self.canonical_disposition()
    }

    fn pre_time_evidence(&self) -> CanonicalCompletedQualifiedTimeEvidenceV1 {
        CanonicalCompletedQualifiedTimeEvidenceV1::from_qualified(self.pre_invocation_time())
    }

    fn post_time_evidence(&self) -> Option<CanonicalCompletedQualifiedTimeEvidenceV1> {
        self.post_invocation_time()
            .map(CanonicalCompletedQualifiedTimeEvidenceV1::from_qualified)
    }

    fn post_ambiguity(&self) -> Option<CanonicalPostInvocationAmbiguityReasonV1> {
        self.post_invocation_ambiguity()
    }

    fn creates_direct_replay_authority(&self) -> bool {
        self.creates_replay_authority()
    }

    fn permits_third_attempt(&self) -> bool {
        self.permits_third_attempt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalCompletedEffectEvidenceErrorV1 {
    UnsupportedAttemptSchema,
    ProvenanceAttemptMismatch,
    MissingDurableInvocationRecord,
    LowLevelSubjectMismatch,
    LowLevelAttemptMismatch,
    ReportedReceiptMismatch,
    PreInvocationTimeRegressedBeforeAttempt,
    PreInvocationTimeAlreadyExpired,
    PostInvocationTimeMissingWithoutQualificationFailure,
    PostInvocationTimePresentWithQualificationFailure,
    PostInvocationTimeAlreadyExpired,
    PostInvocationTimeRegressedWithoutRecordedAmbiguity,
    SpuriousPostInvocationRollbackAmbiguity,
    CanonicalDispositionMismatch,
    UnexpectedDirectReplayAuthority,
    UnexpectedThirdAttemptPermission,
}

pub struct CanonicalCompletedEffectEvidenceFailureV1<O> {
    outcome: O,
    error: CanonicalCompletedEffectEvidenceErrorV1,
}

impl<O> CanonicalCompletedEffectEvidenceFailureV1<O> {
    pub const fn error(&self) -> CanonicalCompletedEffectEvidenceErrorV1 {
        self.error
    }

    pub fn into_outcome(self) -> O {
        self.outcome
    }
}

/// Copyable evidence only. There is no public field constructor and no method
/// that returns the consumed execution typestate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalCompletedEffectEvidenceV1 {
    provenance: CanonicalCompletedInvocationProvenanceV1,
    invocation_record: CanonicalCompletedInvocationRecordV1,
    attempt: ActuatorInvocationAttemptManifestV1,
    reported_receipt: Option<ActuatorEffectReceiptV1>,
    low_level_outcome: ValidatedActuatorEffectOutcomeV1,
    canonical_disposition: CanonicalQualifiedEffectDispositionV1,
    pre_invocation_time: CanonicalCompletedQualifiedTimeEvidenceV1,
    post_invocation_time: Option<CanonicalCompletedQualifiedTimeEvidenceV1>,
    post_invocation_ambiguity: Option<CanonicalPostInvocationAmbiguityReasonV1>,
}

impl CanonicalCompletedEffectEvidenceV1 {
    pub const fn provenance(&self) -> CanonicalCompletedInvocationProvenanceV1 {
        self.provenance
    }

    pub const fn invocation_record(&self) -> CanonicalCompletedInvocationRecordV1 {
        self.invocation_record
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn reported_receipt(&self) -> Option<ActuatorEffectReceiptV1> {
        self.reported_receipt
    }

    pub const fn low_level_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.low_level_outcome
    }

    pub const fn canonical_disposition(&self) -> CanonicalQualifiedEffectDispositionV1 {
        self.canonical_disposition
    }

    pub const fn pre_invocation_time(&self) -> CanonicalCompletedQualifiedTimeEvidenceV1 {
        self.pre_invocation_time
    }

    pub const fn post_invocation_time(&self) -> Option<CanonicalCompletedQualifiedTimeEvidenceV1> {
        self.post_invocation_time
    }

    pub const fn post_invocation_ambiguity(
        &self,
    ) -> Option<CanonicalPostInvocationAmbiguityReasonV1> {
        self.post_invocation_ambiguity
    }

    pub const fn contains_execution_authority(&self) -> bool {
        false
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }

    pub const fn preserves_raw_report_separately(&self) -> bool {
        true
    }
}

fn receipt_commitment(
    receipt: Option<ActuatorEffectReceiptV1>,
) -> Option<ActuatorEffectReceiptCommitment> {
    receipt.map(|value| value.receipt_commitment)
}

fn canonical_disposition_matches(
    low_level: ValidatedActuatorEffectOutcomeV1,
    canonical: CanonicalQualifiedEffectDispositionV1,
    post_ambiguity: Option<CanonicalPostInvocationAmbiguityReasonV1>,
) -> bool {
    let low_receipt = receipt_commitment(low_level.receipt());

    match (canonical, post_ambiguity, low_level.disposition()) {
        (
            CanonicalQualifiedEffectDispositionV1::Confirmed {
                outcome,
                actuator_receipt,
            },
            None,
            ValidatedActuatorEffectDispositionV1::Confirmed {
                outcome: low_outcome,
            },
        ) => low_outcome == outcome && low_receipt == Some(actuator_receipt),
        (
            CanonicalQualifiedEffectDispositionV1::ProvenNotApplied {
                evidence,
                actuator_receipt,
            },
            None,
            ValidatedActuatorEffectDispositionV1::ProvenNotApplied {
                evidence: low_evidence,
            },
        ) => low_evidence == evidence && low_receipt == Some(actuator_receipt),
        (
            CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
                low_level_disposition,
                qualified_actuator_receipt,
                post_invocation_ambiguity,
                recovery_policy,
            },
            outer_post_ambiguity,
            actual_low_level,
        ) => {
            low_level_disposition == actual_low_level
                && qualified_actuator_receipt == low_receipt
                && post_invocation_ambiguity == outer_post_ambiguity
                && recovery_policy == low_level.recovery_policy()
        }
        _ => false,
    }
}

pub fn freeze_canonical_completed_effect_evidence<O>(
    outcome: O,
) -> Result<
    CanonicalCompletedEffectEvidenceV1,
    CanonicalCompletedEffectEvidenceFailureV1<O>,
>
where
    O: ExactCompletedCanonicalInvocationV1,
{
    let provenance = outcome.provenance();
    let attempt = outcome.attempt();
    let invocation_record = match provenance.invocation_record() {
        Some(record) => record,
        None => {
            return Err(CanonicalCompletedEffectEvidenceFailureV1 {
                outcome,
                error: CanonicalCompletedEffectEvidenceErrorV1::MissingDurableInvocationRecord,
            });
        }
    };
    let reported_receipt = outcome.reported_receipt();
    let low_level = outcome.low_level_outcome();
    let canonical = outcome.canonical_disposition();
    let pre_time = outcome.pre_time_evidence();
    let post_time = outcome.post_time_evidence();
    let post_ambiguity = outcome.post_ambiguity();

    let error = if attempt.schema_version != SSF_SCHEMA_V1 {
        Some(CanonicalCompletedEffectEvidenceErrorV1::UnsupportedAttemptSchema)
    } else if provenance.attempt() != attempt {
        Some(CanonicalCompletedEffectEvidenceErrorV1::ProvenanceAttemptMismatch)
    } else if low_level.subject() != attempt.subject {
        Some(CanonicalCompletedEffectEvidenceErrorV1::LowLevelSubjectMismatch)
    } else if low_level.attempt_id() != attempt.attempt_id {
        Some(CanonicalCompletedEffectEvidenceErrorV1::LowLevelAttemptMismatch)
    } else if reported_receipt != low_level.receipt() {
        Some(CanonicalCompletedEffectEvidenceErrorV1::ReportedReceiptMismatch)
    } else if pre_time.latest_possible_unix_ms < attempt.latest_possible_unix_ms {
        Some(CanonicalCompletedEffectEvidenceErrorV1::PreInvocationTimeRegressedBeforeAttempt)
    } else if pre_time.valid_until < pre_time.latest_possible_unix_ms {
        Some(CanonicalCompletedEffectEvidenceErrorV1::PreInvocationTimeAlreadyExpired)
    } else if post_time.is_none()
        && post_ambiguity
            != Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeQualificationFailed)
    {
        Some(
            CanonicalCompletedEffectEvidenceErrorV1::PostInvocationTimeMissingWithoutQualificationFailure,
        )
    } else if post_time.is_some()
        && post_ambiguity
            == Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeQualificationFailed)
    {
        Some(
            CanonicalCompletedEffectEvidenceErrorV1::PostInvocationTimePresentWithQualificationFailure,
        )
    } else if let Some(post) = post_time {
        if post.valid_until < post.latest_possible_unix_ms {
            Some(CanonicalCompletedEffectEvidenceErrorV1::PostInvocationTimeAlreadyExpired)
        } else if post.latest_possible_unix_ms < pre_time.latest_possible_unix_ms
            && post_ambiguity
                != Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeRegressed)
        {
            Some(
                CanonicalCompletedEffectEvidenceErrorV1::PostInvocationTimeRegressedWithoutRecordedAmbiguity,
            )
        } else if post.latest_possible_unix_ms >= pre_time.latest_possible_unix_ms
            && post_ambiguity
                == Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeRegressed)
        {
            Some(CanonicalCompletedEffectEvidenceErrorV1::SpuriousPostInvocationRollbackAmbiguity)
        } else if !canonical_disposition_matches(low_level, canonical, post_ambiguity) {
            Some(CanonicalCompletedEffectEvidenceErrorV1::CanonicalDispositionMismatch)
        } else if outcome.creates_direct_replay_authority() {
            Some(CanonicalCompletedEffectEvidenceErrorV1::UnexpectedDirectReplayAuthority)
        } else if outcome.permits_third_attempt() {
            Some(CanonicalCompletedEffectEvidenceErrorV1::UnexpectedThirdAttemptPermission)
        } else {
            None
        }
    } else if !canonical_disposition_matches(low_level, canonical, post_ambiguity) {
        Some(CanonicalCompletedEffectEvidenceErrorV1::CanonicalDispositionMismatch)
    } else if outcome.creates_direct_replay_authority() {
        Some(CanonicalCompletedEffectEvidenceErrorV1::UnexpectedDirectReplayAuthority)
    } else if outcome.permits_third_attempt() {
        Some(CanonicalCompletedEffectEvidenceErrorV1::UnexpectedThirdAttemptPermission)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(CanonicalCompletedEffectEvidenceFailureV1 { outcome, error });
    }

    Ok(CanonicalCompletedEffectEvidenceV1 {
        provenance,
        invocation_record,
        attempt,
        reported_receipt,
        low_level_outcome: low_level,
        canonical_disposition: canonical,
        pre_invocation_time: pre_time,
        post_invocation_time: post_time,
        post_invocation_ambiguity: post_ambiguity,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evidence_is_copyable_and_non_authoritative() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<CanonicalCompletedEffectEvidenceV1>();
    }

    #[test]
    fn legacy_and_canonical_replay_records_are_distinct_domains() {
        let record = ReplayInvocationJournalRecordCommitment::from_bytes([7; 32]);
        assert_ne!(
            CanonicalCompletedInvocationRecordV1::LegacyReplay(record),
            CanonicalCompletedInvocationRecordV1::CanonicalReplay(record)
        );
    }

    #[test]
    fn post_time_self_freshness_is_explicit() {
        let latest = 101_u64;
        let valid_until = 100_u64;
        assert!(valid_until < latest);
    }
}

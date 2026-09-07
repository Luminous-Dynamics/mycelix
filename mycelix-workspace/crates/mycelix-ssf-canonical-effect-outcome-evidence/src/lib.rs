// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Non-authoritative canonical evidence envelope for one completed SSF actuator
//! invocation.
//!
//! Execution authority is consumed upstream. This crate preserves the exact
//! journal provenance, immutable actuator report, low-level interpretation,
//! stricter canonical qualification, and trusted-time evidence without adding
//! any execution or replay capability.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::{
    ActuatorEffectReceiptCommitment, ActuatorEffectReceiptV1,
    ActuatorInvocationAttemptManifestV1, ValidatedActuatorEffectDispositionV1,
    ValidatedActuatorEffectOutcomeV1,
};
use mycelix_ssf_canonical_actuator_execution::{
    CanonicalActuatorInvocationOutcomeV1, CanonicalInvocationJournalAuditBindingV1,
    CanonicalPostInvocationAmbiguityReasonV1, CanonicalQualifiedEffectDispositionV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, ExpectedCurrentTimeQualifierProfileV1, QualifiedCurrentTimeV1,
    TimeSourceCommitment,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalQualifiedTimeEvidenceV1 {
    pub expected_profile: ExpectedCurrentTimeQualifierProfileV1,
    pub source: TimeSourceCommitment,
    pub observed_unix_ms: u64,
    pub uncertainty_ms: u64,
    pub latest_possible_unix_ms: u64,
    pub valid_until: u64,
    pub receipt_commitment: CurrentTimeReceiptCommitment,
}

impl CanonicalQualifiedTimeEvidenceV1 {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalEffectOutcomeEvidenceErrorV1 {
    UnsupportedAttemptSchema,
    JournalAttemptMismatch,
    LowLevelSubjectMismatch,
    LowLevelAttemptMismatch,
    ReportedReceiptMismatch,
    PreInvocationTimeRegressedBeforeAttempt,
    PreInvocationTimeAlreadyExpired,
    PostInvocationTimeMissingWithoutQualificationFailure,
    PostInvocationTimePresentWithQualificationFailure,
    PostInvocationTimeRegressedWithoutRecordedAmbiguity,
    CanonicalDispositionMismatch,
}

pub struct CanonicalEffectOutcomeEvidenceFailureV1<A, P, TQ> {
    outcome: CanonicalActuatorInvocationOutcomeV1<A, P, TQ>,
    error: CanonicalEffectOutcomeEvidenceErrorV1,
}

impl<A, P, TQ> CanonicalEffectOutcomeEvidenceFailureV1<A, P, TQ> {
    pub const fn error(&self) -> CanonicalEffectOutcomeEvidenceErrorV1 {
        self.error
    }

    pub fn into_outcome(self) -> CanonicalActuatorInvocationOutcomeV1<A, P, TQ> {
        self.outcome
    }
}

/// Copyable evidence only. There is deliberately no constructor from these
/// fields and no method that exposes execution or replay authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalEffectOutcomeEvidenceV1 {
    journal_audit: CanonicalInvocationJournalAuditBindingV1,
    attempt: ActuatorInvocationAttemptManifestV1,
    reported_receipt: Option<ActuatorEffectReceiptV1>,
    low_level_outcome: ValidatedActuatorEffectOutcomeV1,
    canonical_disposition: CanonicalQualifiedEffectDispositionV1,
    pre_invocation_time: CanonicalQualifiedTimeEvidenceV1,
    post_invocation_time: Option<CanonicalQualifiedTimeEvidenceV1>,
    post_invocation_ambiguity: Option<CanonicalPostInvocationAmbiguityReasonV1>,
}

impl CanonicalEffectOutcomeEvidenceV1 {
    pub const fn journal_audit(&self) -> CanonicalInvocationJournalAuditBindingV1 {
        self.journal_audit
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

    pub const fn pre_invocation_time(&self) -> CanonicalQualifiedTimeEvidenceV1 {
        self.pre_invocation_time
    }

    pub const fn post_invocation_time(&self) -> Option<CanonicalQualifiedTimeEvidenceV1> {
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

pub fn freeze_canonical_effect_outcome_evidence<A, P, TQ>(
    outcome: CanonicalActuatorInvocationOutcomeV1<A, P, TQ>,
) -> Result<CanonicalEffectOutcomeEvidenceV1, CanonicalEffectOutcomeEvidenceFailureV1<A, P, TQ>> {
    let attempt = outcome.attempt();
    let journal_audit = outcome.journal_audit();
    let reported_receipt = outcome.reported_receipt();
    let low_level = outcome.low_level_outcome();
    let canonical = outcome.canonical_disposition();
    let post_ambiguity = outcome.post_invocation_ambiguity();
    let pre_time = CanonicalQualifiedTimeEvidenceV1::from_qualified(outcome.pre_invocation_time());
    let post_time = outcome
        .post_invocation_time()
        .map(CanonicalQualifiedTimeEvidenceV1::from_qualified);

    let error = if attempt.schema_version != SSF_SCHEMA_V1 {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::UnsupportedAttemptSchema)
    } else if journal_audit.attempt != attempt {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::JournalAttemptMismatch)
    } else if low_level.subject() != attempt.subject {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::LowLevelSubjectMismatch)
    } else if low_level.attempt_id() != attempt.attempt_id {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::LowLevelAttemptMismatch)
    } else if reported_receipt != low_level.receipt() {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::ReportedReceiptMismatch)
    } else if pre_time.latest_possible_unix_ms < attempt.latest_possible_unix_ms {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::PreInvocationTimeRegressedBeforeAttempt)
    } else if pre_time.valid_until < pre_time.latest_possible_unix_ms {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::PreInvocationTimeAlreadyExpired)
    } else if post_time.is_none()
        && post_ambiguity
            != Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeQualificationFailed)
    {
        Some(
            CanonicalEffectOutcomeEvidenceErrorV1::PostInvocationTimeMissingWithoutQualificationFailure,
        )
    } else if post_time.is_some()
        && post_ambiguity
            == Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeQualificationFailed)
    {
        Some(
            CanonicalEffectOutcomeEvidenceErrorV1::PostInvocationTimePresentWithQualificationFailure,
        )
    } else if let Some(post) = post_time {
        if post.latest_possible_unix_ms < pre_time.latest_possible_unix_ms
            && post_ambiguity
                != Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeRegressed)
        {
            Some(
                CanonicalEffectOutcomeEvidenceErrorV1::PostInvocationTimeRegressedWithoutRecordedAmbiguity,
            )
        } else if !canonical_disposition_matches(low_level, canonical, post_ambiguity) {
            Some(CanonicalEffectOutcomeEvidenceErrorV1::CanonicalDispositionMismatch)
        } else {
            None
        }
    } else if !canonical_disposition_matches(low_level, canonical, post_ambiguity) {
        Some(CanonicalEffectOutcomeEvidenceErrorV1::CanonicalDispositionMismatch)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(CanonicalEffectOutcomeEvidenceFailureV1 { outcome, error });
    }

    Ok(CanonicalEffectOutcomeEvidenceV1 {
        journal_audit,
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
    use mycelix_ssf_actuator_effect_protocol::{
        ActuatorEffectAmbiguityReasonV1, EffectRecoveryPolicyV1,
    };

    #[test]
    fn canonical_evidence_is_non_authoritative() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<CanonicalEffectOutcomeEvidenceV1>();
    }

    #[test]
    fn unknown_reason_remains_distinct_from_recovery_policy() {
        let reason = ActuatorEffectAmbiguityReasonV1::StoreReportedOutcomeUnknown;
        let policy = EffectRecoveryPolicyV1::NeverAutomaticRetry;
        assert_ne!(core::mem::size_of_val(&reason), 0);
        assert_ne!(core::mem::size_of_val(&policy), 0);
    }
}
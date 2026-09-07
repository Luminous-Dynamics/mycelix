// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fresh replay-evidence qualification over canonical durable outcome history.
//!
//! A fresh store read does not by itself refresh old evidence. This crate makes
//! the structural replay basis explicit, then requires an independently expected
//! evidence qualifier to attest that the exact canonical history entry is fit for
//! later replay-policy consideration. No replay authority is created here.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    recovery_policy_for, ActuatorEffectSubjectV1, ActuatorInvocationAttemptId,
    ActuatorStableEffectIdentityV1, EffectRecoveryPolicyV1,
};
use mycelix_ssf_canonical_actuator_execution::CanonicalQualifiedEffectDispositionV1;
use mycelix_ssf_canonical_effect_outcome_history::{
    CanonicalOutcomeHistoryEntryV1, CanonicalOutcomeHistoryHeadReceiptCommitment,
    CanonicalOutcomeHistoryHeadV1, CanonicalOutcomeStoreTimeBasisV1,
    QualifiedCanonicalOutcomeHistoryHeadV1,
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

digest_type!(ReplayEvidenceQualifierIdentityCommitment);
digest_type!(ReplayEvidenceQualificationPolicyCommitment);
digest_type!(ReplayEvidenceQualificationReceiptCommitment);
digest_type!(ReplayEvidenceQualificationCommitment);

generation_type!(ReplayEvidenceQualifierGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplayEvidenceQualifierTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ReplayEvidenceQualifierDescriptorV1 {
    pub stable_identity: ReplayEvidenceQualifierIdentityCommitment,
    pub policy: ReplayEvidenceQualificationPolicyCommitment,
    pub generation: ReplayEvidenceQualifierGeneration,
    pub time_basis: ReplayEvidenceQualifierTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedReplayEvidenceQualifierProfileV1 {
    descriptor: ReplayEvidenceQualifierDescriptorV1,
}

impl ExpectedReplayEvidenceQualifierProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: ReplayEvidenceQualifierDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> ReplayEvidenceQualifierDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceBasisV1 {
    ProvenNotApplied,
    IdempotentOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayEvidenceQualificationSubjectV1 {
    pub history_head: CanonicalOutcomeHistoryHeadV1,
    pub history_head_receipt: CanonicalOutcomeHistoryHeadReceiptCommitment,
    pub latest_entry: CanonicalOutcomeHistoryEntryV1,
    pub history_read_time_receipt: CurrentTimeReceiptCommitment,
    pub history_read_latest_possible_unix_ms: u64,
    pub qualification_time_receipt: CurrentTimeReceiptCommitment,
    pub qualification_latest_possible_unix_ms: u64,
    pub prior_attempt_id: ActuatorInvocationAttemptId,
    pub prior_effect_subject: ActuatorEffectSubjectV1,
    pub stable_effect_identity: ActuatorStableEffectIdentityV1,
    pub basis: CanonicalReplayEvidenceBasisV1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayEvidenceQualificationDispositionV1 {
    Qualified {
        qualification: ReplayEvidenceQualificationCommitment,
        evidence_valid_until: u64,
    },
    Reject,
    Defer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayEvidenceQualificationReceiptV1 {
    pub schema_version: u16,
    pub qualifier: ReplayEvidenceQualifierDescriptorV1,
    pub subject: CanonicalReplayEvidenceQualificationSubjectV1,
    pub disposition: ReplayEvidenceQualificationDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: ReplayEvidenceQualificationReceiptCommitment,
}

pub trait ReplayEvidenceQualifierV1 {
    type Error;

    fn descriptor(&self) -> ReplayEvidenceQualifierDescriptorV1;

    fn qualify_replay_evidence(
        &self,
        subject: &CanonicalReplayEvidenceQualificationSubjectV1,
    ) -> Result<ReplayEvidenceQualificationReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReplayEvidenceStructuralErrorV1 {
    HistoryStoreUsesUnsupportedTimeBasis,
    HistoryReadAlreadyExpired,
    QualificationTimeAlreadyExpired,
    QualificationTimeRegressedBeforeHistoryRead,
    HistoryEmpty,
    LatestEntryMissing,
    LatestEntryAttemptMismatch,
    ConfirmedEffectCannotReplay,
    TransactionalUnknownRequiresReconciliation,
    NonIdempotentUnknownCannotReplay,
    UnknownRecoveryPolicyMismatch,
}

fn replay_basis(
    disposition: CanonicalQualifiedEffectDispositionV1,
    prior_effect_subject: ActuatorEffectSubjectV1,
) -> Result<CanonicalReplayEvidenceBasisV1, CanonicalReplayEvidenceStructuralErrorV1> {
    match disposition {
        CanonicalQualifiedEffectDispositionV1::Confirmed { .. } => {
            Err(CanonicalReplayEvidenceStructuralErrorV1::ConfirmedEffectCannotReplay)
        }
        CanonicalQualifiedEffectDispositionV1::ProvenNotApplied { .. } => {
            Ok(CanonicalReplayEvidenceBasisV1::ProvenNotApplied)
        }
        CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
            recovery_policy, ..
        } => {
            if recovery_policy != recovery_policy_for(prior_effect_subject.recovery_mode) {
                return Err(
                    CanonicalReplayEvidenceStructuralErrorV1::UnknownRecoveryPolicyMismatch,
                );
            }
            match recovery_policy {
                EffectRecoveryPolicyV1::ReconcileOnly => Err(
                    CanonicalReplayEvidenceStructuralErrorV1::TransactionalUnknownRequiresReconciliation,
                ),
                EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted => {
                    Ok(CanonicalReplayEvidenceBasisV1::IdempotentOutcomeUnknown)
                }
                EffectRecoveryPolicyV1::NeverAutomaticRetry => Err(
                    CanonicalReplayEvidenceStructuralErrorV1::NonIdempotentUnknownCannotReplay,
                ),
            }
        }
    }
}

pub struct CanonicalReplayEvidenceStructuralFailureV1<S, HTQ, QTQ> {
    history: QualifiedCanonicalOutcomeHistoryHeadV1<S, HTQ>,
    qualification_time: QualifiedCurrentTimeV1<QTQ>,
    error: CanonicalReplayEvidenceStructuralErrorV1,
}

impl<S, HTQ, QTQ> CanonicalReplayEvidenceStructuralFailureV1<S, HTQ, QTQ> {
    pub const fn error(&self) -> CanonicalReplayEvidenceStructuralErrorV1 {
        self.error
    }

    pub fn into_parts(
        self,
    ) -> (
        QualifiedCanonicalOutcomeHistoryHeadV1<S, HTQ>,
        QualifiedCurrentTimeV1<QTQ>,
    ) {
        (self.history, self.qualification_time)
    }
}

pub struct CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ> {
    history: QualifiedCanonicalOutcomeHistoryHeadV1<S, HTQ>,
    qualification_time: QualifiedCurrentTimeV1<QTQ>,
    subject: CanonicalReplayEvidenceQualificationSubjectV1,
}

impl<S, HTQ, QTQ> CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ> {
    pub const fn history(&self) -> &QualifiedCanonicalOutcomeHistoryHeadV1<S, HTQ> {
        &self.history
    }

    pub const fn qualification_time(&self) -> &QualifiedCurrentTimeV1<QTQ> {
        &self.qualification_time
    }

    pub const fn subject(&self) -> CanonicalReplayEvidenceQualificationSubjectV1 {
        self.subject
    }

    pub const fn contains_replay_authority(&self) -> bool {
        false
    }
}

pub fn prepare_canonical_replay_evidence_candidate<S, HTQ, QTQ>(
    history: QualifiedCanonicalOutcomeHistoryHeadV1<S, HTQ>,
    qualification_time: QualifiedCurrentTimeV1<QTQ>,
) -> Result<
    CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ>,
    CanonicalReplayEvidenceStructuralFailureV1<S, HTQ, QTQ>,
> {
    let receipt = history.receipt();
    let history_read_latest = history.current_time().latest_possible_unix_ms();
    let qualification_latest = qualification_time.latest_possible_unix_ms();

    let fail = |history,
                qualification_time,
                error| CanonicalReplayEvidenceStructuralFailureV1 {
        history,
        qualification_time,
        error,
    };

    if receipt.store.time_basis != CanonicalOutcomeStoreTimeBasisV1::UnixMillisecondsUtc {
        return Err(fail(
            history,
            qualification_time,
            CanonicalReplayEvidenceStructuralErrorV1::HistoryStoreUsesUnsupportedTimeBasis,
        ));
    }
    if receipt.valid_until < qualification_latest {
        return Err(fail(
            history,
            qualification_time,
            CanonicalReplayEvidenceStructuralErrorV1::HistoryReadAlreadyExpired,
        ));
    }
    if qualification_time.valid_until() < qualification_latest {
        return Err(fail(
            history,
            qualification_time,
            CanonicalReplayEvidenceStructuralErrorV1::QualificationTimeAlreadyExpired,
        ));
    }
    if qualification_latest < history_read_latest {
        return Err(fail(
            history,
            qualification_time,
            CanonicalReplayEvidenceStructuralErrorV1::QualificationTimeRegressedBeforeHistoryRead,
        ));
    }
    if receipt.head.generation().get() == 0 {
        return Err(fail(
            history,
            qualification_time,
            CanonicalReplayEvidenceStructuralErrorV1::HistoryEmpty,
        ));
    }

    let latest_entry = match receipt.latest_entry {
        Some(entry) => entry,
        None => {
            return Err(fail(
                history,
                qualification_time,
                CanonicalReplayEvidenceStructuralErrorV1::LatestEntryMissing,
            ));
        }
    };
    let evidence = latest_entry.manifest.subject.evidence;
    let prior_attempt = evidence.attempt();
    if latest_entry.manifest.subject.invocation_record != receipt.invocation_record
        || evidence.journal_audit().attempt != prior_attempt
    {
        return Err(fail(
            history,
            qualification_time,
            CanonicalReplayEvidenceStructuralErrorV1::LatestEntryAttemptMismatch,
        ));
    }

    let basis = match replay_basis(evidence.canonical_disposition(), prior_attempt.subject) {
        Ok(value) => value,
        Err(error) => return Err(fail(history, qualification_time, error)),
    };

    let subject = CanonicalReplayEvidenceQualificationSubjectV1 {
        history_head: receipt.head,
        history_head_receipt: receipt.receipt_commitment,
        latest_entry,
        history_read_time_receipt: history.current_time().receipt_commitment(),
        history_read_latest_possible_unix_ms: history_read_latest,
        qualification_time_receipt: qualification_time.receipt_commitment(),
        qualification_latest_possible_unix_ms: qualification_latest,
        prior_attempt_id: prior_attempt.attempt_id,
        prior_effect_subject: prior_attempt.subject,
        stable_effect_identity: prior_attempt.subject.stable_identity(),
        basis,
    };

    Ok(CanonicalReplayEvidenceCandidateV1 {
        history,
        qualification_time,
        subject,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplayEvidenceQualificationFailureReasonV1 {
    UnsupportedQualifierTimeBasis,
    QualifierAlreadyExpired,
    QualifierDescriptorMismatchBefore,
    QualifierDescriptorChangedAfter,
    QualifierError,
    UnsupportedReceiptSchema,
    ReceiptQualifierMismatch,
    ReceiptSubjectMismatch,
    ReceiptOutlivesQualifier,
    ReceiptOutlivesHistoryRead,
    ReceiptOutlivesQualificationTime,
    ReceiptAlreadyExpired,
    EvidenceValidityOutlivesReceipt,
    EvidenceAlreadyExpired,
}

pub struct ReplayEvidenceQualificationFailureV1<Q, S, HTQ, QTQ> {
    candidate: CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ>,
    expected_qualifier: ExpectedReplayEvidenceQualifierProfileV1,
    reason: ReplayEvidenceQualificationFailureReasonV1,
    _qualifier: PhantomData<fn() -> Q>,
}

impl<Q, S, HTQ, QTQ> ReplayEvidenceQualificationFailureV1<Q, S, HTQ, QTQ> {
    pub const fn reason(&self) -> ReplayEvidenceQualificationFailureReasonV1 {
        self.reason
    }

    pub fn into_candidate(self) -> CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ> {
        self.candidate
    }

    pub const fn expected_qualifier(&self) -> ExpectedReplayEvidenceQualifierProfileV1 {
        self.expected_qualifier
    }
}

pub struct ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ> {
    candidate: CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ>,
    expected_qualifier: ExpectedReplayEvidenceQualifierProfileV1,
    receipt: ReplayEvidenceQualificationReceiptV1,
    qualification: ReplayEvidenceQualificationCommitment,
    valid_until: u64,
    _qualifier: PhantomData<fn() -> Q>,
}

impl<Q, S, HTQ, QTQ> ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ> {
    pub const fn subject(&self) -> CanonicalReplayEvidenceQualificationSubjectV1 {
        self.candidate.subject
    }

    pub const fn receipt(&self) -> ReplayEvidenceQualificationReceiptV1 {
        self.receipt
    }

    pub const fn qualification(&self) -> ReplayEvidenceQualificationCommitment {
        self.qualification
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn expected_qualifier(&self) -> ExpectedReplayEvidenceQualifierProfileV1 {
        self.expected_qualifier
    }

    pub const fn contains_replay_authority(&self) -> bool {
        false
    }
}

pub struct RejectedCanonicalReplayEvidenceV1<S, HTQ, QTQ> {
    candidate: CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ>,
    receipt: ReplayEvidenceQualificationReceiptV1,
}

impl<S, HTQ, QTQ> RejectedCanonicalReplayEvidenceV1<S, HTQ, QTQ> {
    pub const fn receipt(&self) -> ReplayEvidenceQualificationReceiptV1 {
        self.receipt
    }

    pub const fn replay_allowed(&self) -> bool {
        false
    }

    pub fn into_candidate(self) -> CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ> {
        self.candidate
    }
}

pub struct DeferredCanonicalReplayEvidenceV1<S, HTQ, QTQ> {
    candidate: CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ>,
    receipt: ReplayEvidenceQualificationReceiptV1,
}

impl<S, HTQ, QTQ> DeferredCanonicalReplayEvidenceV1<S, HTQ, QTQ> {
    pub const fn receipt(&self) -> ReplayEvidenceQualificationReceiptV1 {
        self.receipt
    }

    pub const fn replay_allowed(&self) -> bool {
        false
    }

    pub fn into_candidate(self) -> CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ> {
        self.candidate
    }
}

pub enum CanonicalReplayEvidenceQualificationDecisionV1<Q, S, HTQ, QTQ> {
    Qualified(ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ>),
    Rejected(RejectedCanonicalReplayEvidenceV1<S, HTQ, QTQ>),
    Deferred(DeferredCanonicalReplayEvidenceV1<S, HTQ, QTQ>),
}

pub fn qualify_canonical_replay_evidence<Q, S, HTQ, QTQ>(
    candidate: CanonicalReplayEvidenceCandidateV1<S, HTQ, QTQ>,
    expected_qualifier: ExpectedReplayEvidenceQualifierProfileV1,
    qualifier: &Q,
) -> Result<
    CanonicalReplayEvidenceQualificationDecisionV1<Q, S, HTQ, QTQ>,
    ReplayEvidenceQualificationFailureV1<Q, S, HTQ, QTQ>,
>
where
    Q: ReplayEvidenceQualifierV1,
{
    let latest = candidate.qualification_time.latest_possible_unix_ms();
    let history_receipt_valid_until = candidate.history.receipt().valid_until;

    let failure = |candidate,
                   expected_qualifier,
                   reason| ReplayEvidenceQualificationFailureV1 {
        candidate,
        expected_qualifier,
        reason,
        _qualifier: PhantomData,
    };

    if expected_qualifier.descriptor.time_basis
        != ReplayEvidenceQualifierTimeBasisV1::UnixMillisecondsUtc
    {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::UnsupportedQualifierTimeBasis,
        ));
    }
    if expected_qualifier.descriptor.valid_until < latest {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::QualifierAlreadyExpired,
        ));
    }
    if qualifier.descriptor() != expected_qualifier.descriptor {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::QualifierDescriptorMismatchBefore,
        ));
    }

    let receipt = match qualifier.qualify_replay_evidence(&candidate.subject) {
        Ok(value) => value,
        Err(_) => {
            return Err(failure(
                candidate,
                expected_qualifier,
                ReplayEvidenceQualificationFailureReasonV1::QualifierError,
            ));
        }
    };
    if qualifier.descriptor() != expected_qualifier.descriptor {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::QualifierDescriptorChangedAfter,
        ));
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::UnsupportedReceiptSchema,
        ));
    }
    if receipt.qualifier != expected_qualifier.descriptor {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::ReceiptQualifierMismatch,
        ));
    }
    if receipt.subject != candidate.subject {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::ReceiptSubjectMismatch,
        ));
    }
    if receipt.valid_until > expected_qualifier.descriptor.valid_until {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::ReceiptOutlivesQualifier,
        ));
    }
    if receipt.valid_until > history_receipt_valid_until {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::ReceiptOutlivesHistoryRead,
        ));
    }
    if receipt.valid_until > candidate.qualification_time.valid_until() {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::ReceiptOutlivesQualificationTime,
        ));
    }
    if receipt.valid_until < latest {
        return Err(failure(
            candidate,
            expected_qualifier,
            ReplayEvidenceQualificationFailureReasonV1::ReceiptAlreadyExpired,
        ));
    }

    match receipt.disposition {
        ReplayEvidenceQualificationDispositionV1::Qualified {
            qualification,
            evidence_valid_until,
        } => {
            if evidence_valid_until > receipt.valid_until {
                return Err(failure(
                    candidate,
                    expected_qualifier,
                    ReplayEvidenceQualificationFailureReasonV1::EvidenceValidityOutlivesReceipt,
                ));
            }
            if evidence_valid_until < latest {
                return Err(failure(
                    candidate,
                    expected_qualifier,
                    ReplayEvidenceQualificationFailureReasonV1::EvidenceAlreadyExpired,
                ));
            }
            let valid_until = evidence_valid_until
                .min(receipt.valid_until)
                .min(history_receipt_valid_until)
                .min(candidate.qualification_time.valid_until())
                .min(expected_qualifier.descriptor.valid_until);
            Ok(CanonicalReplayEvidenceQualificationDecisionV1::Qualified(
                ReplayQualifiedCanonicalOutcomeEvidenceV1 {
                    candidate,
                    expected_qualifier,
                    receipt,
                    qualification,
                    valid_until,
                    _qualifier: PhantomData,
                },
            ))
        }
        ReplayEvidenceQualificationDispositionV1::Reject => Ok(
            CanonicalReplayEvidenceQualificationDecisionV1::Rejected(
                RejectedCanonicalReplayEvidenceV1 { candidate, receipt },
            ),
        ),
        ReplayEvidenceQualificationDispositionV1::Defer => Ok(
            CanonicalReplayEvidenceQualificationDecisionV1::Deferred(
                DeferredCanonicalReplayEvidenceV1 { candidate, receipt },
            ),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn confirmed_is_structural_stop() {
        let disposition = CanonicalQualifiedEffectDispositionV1::Confirmed {
            outcome: mycelix_ssf_actuator_effect_protocol::ActuatorEffectOutcomeCommitment::from_bytes([1; 32]),
            actuator_receipt:
                mycelix_ssf_actuator_effect_protocol::ActuatorEffectReceiptCommitment::from_bytes([2; 32]),
        };
        let _ = disposition;
        assert!(matches!(
            disposition,
            CanonicalQualifiedEffectDispositionV1::Confirmed { .. }
        ));
    }

    #[test]
    fn evidence_qualification_is_not_replay_authority() {
        let basis = CanonicalReplayEvidenceBasisV1::ProvenNotApplied;
        assert_eq!(basis, CanonicalReplayEvidenceBasisV1::ProvenNotApplied);
    }
}
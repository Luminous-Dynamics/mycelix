// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Opaque, non-authoritative replay evidence subject.
//!
//! This crate consumes the structurally eligible first-attempt replay evidence
//! candidate and freezes all evidence a future qualifier is allowed to inspect
//! into one copyable subject. Construction is intentionally unavailable from
//! public fields, preventing evidence-substitution after provenance gating.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::{
    ActuatorEffectSubjectV1, ActuatorInvocationAttemptId, ActuatorStableEffectIdentityV1,
};
use mycelix_ssf_canonical_completed_effect_evidence::CanonicalCompletedInvocationRecordV1;
use mycelix_ssf_canonical_completed_outcome_history::{
    CanonicalCompletedOutcomeHeadReceiptCommitment, CanonicalCompletedOutcomeHistoryEntryV1,
    CanonicalCompletedOutcomeHistoryHeadV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::CurrentTimeReceiptCommitment;
use mycelix_ssf_no_third_attempt_provenance_gate::{
    InitialReplayEvidenceBasisV1, InitialReplayEvidenceCandidateV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceSubjectTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayEvidenceSubjectV1 {
    schema_version: u16,
    time_basis: CanonicalReplayEvidenceSubjectTimeBasisV1,
    invocation_record: CanonicalCompletedInvocationRecordV1,
    history_head: CanonicalCompletedOutcomeHistoryHeadV1,
    history_head_receipt: CanonicalCompletedOutcomeHeadReceiptCommitment,
    latest_entry: CanonicalCompletedOutcomeHistoryEntryV1,
    history_read_time_receipt: CurrentTimeReceiptCommitment,
    history_read_latest_possible_unix_ms: u64,
    prior_attempt_id: ActuatorInvocationAttemptId,
    prior_effect_subject: ActuatorEffectSubjectV1,
    stable_effect_identity: ActuatorStableEffectIdentityV1,
    basis: InitialReplayEvidenceBasisV1,
    valid_until: u64,
}

impl CanonicalReplayEvidenceSubjectV1 {
    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub const fn time_basis(&self) -> CanonicalReplayEvidenceSubjectTimeBasisV1 {
        self.time_basis
    }

    pub const fn invocation_record(&self) -> CanonicalCompletedInvocationRecordV1 {
        self.invocation_record
    }

    pub const fn history_head(&self) -> CanonicalCompletedOutcomeHistoryHeadV1 {
        self.history_head
    }

    pub const fn history_head_receipt(&self) -> CanonicalCompletedOutcomeHeadReceiptCommitment {
        self.history_head_receipt
    }

    pub const fn latest_entry(&self) -> CanonicalCompletedOutcomeHistoryEntryV1 {
        self.latest_entry
    }

    pub const fn history_read_time_receipt(&self) -> CurrentTimeReceiptCommitment {
        self.history_read_time_receipt
    }

    pub const fn history_read_latest_possible_unix_ms(&self) -> u64 {
        self.history_read_latest_possible_unix_ms
    }

    pub const fn prior_attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.prior_attempt_id
    }

    pub const fn prior_effect_subject(&self) -> ActuatorEffectSubjectV1 {
        self.prior_effect_subject
    }

    pub const fn stable_effect_identity(&self) -> ActuatorStableEffectIdentityV1 {
        self.stable_effect_identity
    }

    pub const fn basis(&self) -> InitialReplayEvidenceBasisV1 {
        self.basis
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn provenance_is_initial(&self) -> bool {
        true
    }

    pub const fn contains_replay_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceSubjectErrorV1 {
    MissingLatestEntry,
    NonInitialInvocationRecord,
    InvocationRecordMismatch,
    PriorAttemptMismatch,
    PriorEffectSubjectMismatch,
    StableEffectIdentityMismatch,
}

pub struct CanonicalReplayEvidenceSubjectFailureV1<S, TQ> {
    candidate: InitialReplayEvidenceCandidateV1<S, TQ>,
    error: CanonicalReplayEvidenceSubjectErrorV1,
}

impl<S, TQ> CanonicalReplayEvidenceSubjectFailureV1<S, TQ> {
    pub const fn error(&self) -> CanonicalReplayEvidenceSubjectErrorV1 {
        self.error
    }

    pub fn into_candidate(self) -> InitialReplayEvidenceCandidateV1<S, TQ> {
        self.candidate
    }
}

fn fail<S, TQ>(
    candidate: InitialReplayEvidenceCandidateV1<S, TQ>,
    error: CanonicalReplayEvidenceSubjectErrorV1,
) -> Result<CanonicalReplayEvidenceSubjectV1, CanonicalReplayEvidenceSubjectFailureV1<S, TQ>> {
    Err(CanonicalReplayEvidenceSubjectFailureV1 { candidate, error })
}

pub fn freeze_canonical_replay_evidence_subject<S, TQ>(
    candidate: InitialReplayEvidenceCandidateV1<S, TQ>,
) -> Result<CanonicalReplayEvidenceSubjectV1, CanonicalReplayEvidenceSubjectFailureV1<S, TQ>> {
    let invocation_record = candidate.history().invocation_record();
    if !matches!(invocation_record, CanonicalCompletedInvocationRecordV1::Initial(_)) {
        return fail(
            candidate,
            CanonicalReplayEvidenceSubjectErrorV1::NonInitialInvocationRecord,
        );
    }

    let latest_entry = match candidate.history().latest_entry() {
        Some(entry) => entry,
        None => {
            return fail(
                candidate,
                CanonicalReplayEvidenceSubjectErrorV1::MissingLatestEntry,
            );
        }
    };

    let evidence = latest_entry.manifest.subject.evidence;
    let attempt = evidence.attempt();

    if latest_entry.manifest.subject.invocation_record != invocation_record
        || evidence.invocation_record() != invocation_record
    {
        return fail(
            candidate,
            CanonicalReplayEvidenceSubjectErrorV1::InvocationRecordMismatch,
        );
    }
    if candidate.prior_attempt_id() != attempt.attempt_id {
        return fail(
            candidate,
            CanonicalReplayEvidenceSubjectErrorV1::PriorAttemptMismatch,
        );
    }
    if candidate.prior_effect_subject() != attempt.subject {
        return fail(
            candidate,
            CanonicalReplayEvidenceSubjectErrorV1::PriorEffectSubjectMismatch,
        );
    }
    if candidate.stable_effect_identity() != attempt.subject.stable_identity() {
        return fail(
            candidate,
            CanonicalReplayEvidenceSubjectErrorV1::StableEffectIdentityMismatch,
        );
    }

    let qualified_history = candidate.history().history();

    Ok(CanonicalReplayEvidenceSubjectV1 {
        schema_version: SSF_SCHEMA_V1,
        time_basis: CanonicalReplayEvidenceSubjectTimeBasisV1::UnixMillisecondsUtc,
        invocation_record,
        history_head: qualified_history.head(),
        history_head_receipt: qualified_history.receipt().receipt_commitment,
        latest_entry,
        history_read_time_receipt: qualified_history.current_time().receipt_commitment(),
        history_read_latest_possible_unix_ms: qualified_history
            .current_time()
            .latest_possible_unix_ms(),
        prior_attempt_id: candidate.prior_attempt_id(),
        prior_effect_subject: candidate.prior_effect_subject(),
        stable_effect_identity: candidate.stable_effect_identity(),
        basis: candidate.basis(),
        valid_until: candidate.valid_until(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subject_errors_are_non_interchangeable() {
        assert_ne!(
            CanonicalReplayEvidenceSubjectErrorV1::PriorAttemptMismatch,
            CanonicalReplayEvidenceSubjectErrorV1::StableEffectIdentityMismatch
        );
    }

    #[test]
    fn subject_time_basis_is_explicit() {
        assert_eq!(
            CanonicalReplayEvidenceSubjectTimeBasisV1::UnixMillisecondsUtc,
            CanonicalReplayEvidenceSubjectTimeBasisV1::UnixMillisecondsUtc
        );
    }
}

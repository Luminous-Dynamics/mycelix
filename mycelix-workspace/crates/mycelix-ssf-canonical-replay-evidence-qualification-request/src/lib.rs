// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fresh, anti-rollback qualification request for canonical replay evidence.
//!
//! This crate binds one opaque replay-evidence subject to a fresh trusted-time
//! observation. It does not qualify evidence, call a verifier, or create replay
//! authority. It only proves when the exact subject is being presented for
//! future qualification and that the request did not move backward in time.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_canonical_replay_evidence_subject::{
    CanonicalReplayEvidenceSubjectTimeBasisV1, CanonicalReplayEvidenceSubjectV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayEvidenceQualificationRequestSubjectV1 {
    subject: CanonicalReplayEvidenceSubjectV1,
    qualification_time_receipt: CurrentTimeReceiptCommitment,
    qualification_latest_possible_unix_ms: u64,
    valid_until: u64,
}

impl CanonicalReplayEvidenceQualificationRequestSubjectV1 {
    pub const fn subject(&self) -> CanonicalReplayEvidenceSubjectV1 {
        self.subject
    }

    pub const fn qualification_time_receipt(&self) -> CurrentTimeReceiptCommitment {
        self.qualification_time_receipt
    }

    pub const fn qualification_latest_possible_unix_ms(&self) -> u64 {
        self.qualification_latest_possible_unix_ms
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
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

pub struct CanonicalReplayEvidenceQualificationRequestV1<TQ> {
    subject: CanonicalReplayEvidenceSubjectV1,
    qualification_time: QualifiedCurrentTimeV1<TQ>,
    request_subject: CanonicalReplayEvidenceQualificationRequestSubjectV1,
}

impl<TQ> CanonicalReplayEvidenceQualificationRequestV1<TQ> {
    pub const fn subject(&self) -> CanonicalReplayEvidenceSubjectV1 {
        self.subject
    }

    pub const fn qualification_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.qualification_time
    }

    pub const fn request_subject(&self) -> CanonicalReplayEvidenceQualificationRequestSubjectV1 {
        self.request_subject
    }

    pub const fn valid_until(&self) -> u64 {
        self.request_subject.valid_until
    }

    pub const fn contains_replay_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceQualificationRequestErrorV1 {
    UnsupportedSubjectSchema,
    UnsupportedSubjectTimeBasis,
    SubjectNotInitialProvenance,
    QualificationTimeAlreadyExpired,
    QualificationTimeRegressedBeforeHistoryRead,
    SubjectAlreadyExpired,
}

pub struct CanonicalReplayEvidenceQualificationRequestFailureV1<TQ> {
    subject: CanonicalReplayEvidenceSubjectV1,
    qualification_time: QualifiedCurrentTimeV1<TQ>,
    error: CanonicalReplayEvidenceQualificationRequestErrorV1,
}

impl<TQ> CanonicalReplayEvidenceQualificationRequestFailureV1<TQ> {
    pub const fn error(&self) -> CanonicalReplayEvidenceQualificationRequestErrorV1 {
        self.error
    }

    pub fn into_parts(self) -> (CanonicalReplayEvidenceSubjectV1, QualifiedCurrentTimeV1<TQ>) {
        (self.subject, self.qualification_time)
    }
}

fn fail<TQ>(
    subject: CanonicalReplayEvidenceSubjectV1,
    qualification_time: QualifiedCurrentTimeV1<TQ>,
    error: CanonicalReplayEvidenceQualificationRequestErrorV1,
) -> Result<
    CanonicalReplayEvidenceQualificationRequestV1<TQ>,
    CanonicalReplayEvidenceQualificationRequestFailureV1<TQ>,
> {
    Err(CanonicalReplayEvidenceQualificationRequestFailureV1 {
        subject,
        qualification_time,
        error,
    })
}

pub fn prepare_canonical_replay_evidence_qualification_request<TQ>(
    subject: CanonicalReplayEvidenceSubjectV1,
    qualification_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    CanonicalReplayEvidenceQualificationRequestV1<TQ>,
    CanonicalReplayEvidenceQualificationRequestFailureV1<TQ>,
> {
    let latest = qualification_time.latest_possible_unix_ms();

    if subject.schema_version() != SSF_SCHEMA_V1 {
        return fail(
            subject,
            qualification_time,
            CanonicalReplayEvidenceQualificationRequestErrorV1::UnsupportedSubjectSchema,
        );
    }
    if subject.time_basis() != CanonicalReplayEvidenceSubjectTimeBasisV1::UnixMillisecondsUtc {
        return fail(
            subject,
            qualification_time,
            CanonicalReplayEvidenceQualificationRequestErrorV1::UnsupportedSubjectTimeBasis,
        );
    }
    if !subject.provenance_is_initial() {
        return fail(
            subject,
            qualification_time,
            CanonicalReplayEvidenceQualificationRequestErrorV1::SubjectNotInitialProvenance,
        );
    }
    if qualification_time.valid_until() < latest {
        return fail(
            subject,
            qualification_time,
            CanonicalReplayEvidenceQualificationRequestErrorV1::QualificationTimeAlreadyExpired,
        );
    }
    if latest < subject.history_read_latest_possible_unix_ms() {
        return fail(
            subject,
            qualification_time,
            CanonicalReplayEvidenceQualificationRequestErrorV1::QualificationTimeRegressedBeforeHistoryRead,
        );
    }
    if subject.valid_until() < latest {
        return fail(
            subject,
            qualification_time,
            CanonicalReplayEvidenceQualificationRequestErrorV1::SubjectAlreadyExpired,
        );
    }

    let valid_until = subject.valid_until().min(qualification_time.valid_until());
    let request_subject = CanonicalReplayEvidenceQualificationRequestSubjectV1 {
        subject,
        qualification_time_receipt: qualification_time.receipt_commitment(),
        qualification_latest_possible_unix_ms: latest,
        valid_until,
    };

    Ok(CanonicalReplayEvidenceQualificationRequestV1 {
        subject,
        qualification_time,
        request_subject,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_basis_and_rollback_are_distinct_failures() {
        assert_ne!(
            CanonicalReplayEvidenceQualificationRequestErrorV1::UnsupportedSubjectTimeBasis,
            CanonicalReplayEvidenceQualificationRequestErrorV1::QualificationTimeRegressedBeforeHistoryRead
        );
    }

    #[test]
    fn rollback_and_expiry_are_distinct_failures() {
        assert_ne!(
            CanonicalReplayEvidenceQualificationRequestErrorV1::QualificationTimeRegressedBeforeHistoryRead,
            CanonicalReplayEvidenceQualificationRequestErrorV1::SubjectAlreadyExpired
        );
    }
}

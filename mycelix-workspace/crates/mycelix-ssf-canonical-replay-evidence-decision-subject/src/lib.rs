// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Purpose-scoped semantic subject for replay evidence qualification.
//!
//! This crate consumes one fresh canonical replay-evidence qualification
//! request and freezes the exact semantic object a future evidence qualifier
//! is allowed to evaluate. It independently re-derives the replay basis from
//! the exact latest canonical outcome and stable actuator recovery mode before
//! freezing the decision subject.
//!
//! This crate adds no cryptographic digest, evidence qualification, replay
//! authority, or effect authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::{recovery_policy_for, EffectRecoveryPolicyV1};
use mycelix_ssf_canonical_actuator_execution::CanonicalQualifiedEffectDispositionV1;
use mycelix_ssf_canonical_completed_effect_evidence::CanonicalCompletedInvocationRecordV1;
use mycelix_ssf_canonical_completed_outcome_history::CanonicalCompletedOutcomeTerminalV1;
use mycelix_ssf_canonical_replay_evidence_qualification_request::{
    CanonicalReplayEvidenceQualificationRequestSubjectV1,
    CanonicalReplayEvidenceQualificationRequestV1,
};
use mycelix_ssf_canonical_replay_evidence_subject::{
    CanonicalReplayEvidenceSubjectTimeBasisV1, CanonicalReplayEvidenceSubjectV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_no_third_attempt_provenance_gate::InitialReplayEvidenceBasisV1;

/// Closed semantic purpose for this qualification decision.
///
/// This domain must be preserved by any future canonical encoder, commitment,
/// signature, or verifier receipt so the same evidence cannot be reused under
/// a different authority purpose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceDecisionDomainV1 {
    EvidenceFitnessForAtMostOneReplay,
}

/// Opaque, copyable semantic decision subject.
///
/// Copyability is safe because this object carries evidence semantics only. It
/// is not a cryptographic commitment and creates no authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayEvidenceDecisionSubjectV1 {
    schema_version: u16,
    domain: CanonicalReplayEvidenceDecisionDomainV1,
    request_subject: CanonicalReplayEvidenceQualificationRequestSubjectV1,
    valid_until: u64,
}

impl CanonicalReplayEvidenceDecisionSubjectV1 {
    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub const fn domain(&self) -> CanonicalReplayEvidenceDecisionDomainV1 {
        self.domain
    }

    pub const fn request_subject(&self) -> CanonicalReplayEvidenceQualificationRequestSubjectV1 {
        self.request_subject
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn contains_cryptographic_commitment(&self) -> bool {
        false
    }

    pub const fn requires_canonical_crypto_binding(&self) -> bool {
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
pub enum CanonicalReplayEvidenceDecisionSubjectErrorV1 {
    UnsupportedSubjectSchema,
    UnsupportedSubjectTimeBasis,
    SubjectNotInitialProvenance,
    HistoryInvocationRecordMismatch,
    HistoryHeadRecordMismatch,
    LatestEvidenceInvocationRecordMismatch,
    PriorAttemptMismatch,
    PriorEffectSubjectMismatch,
    StableEffectIdentityMismatch,
    ConfirmedEffectCannotReplay,
    RecoveryPolicyMismatch,
    TransactionalUnknownCannotReplay,
    NonIdempotentUnknownCannotReplay,
    ReplayBasisMismatch,
    HistoryTerminalMismatch,
    RequestSubjectMismatch,
    QualificationTimeReceiptMismatch,
    QualificationTimeMismatch,
    NaturalExpiryMismatch,
    RequestAlreadyExpired,
    UnexpectedAuthority,
}

pub struct CanonicalReplayEvidenceDecisionSubjectFailureV1<TQ> {
    request: CanonicalReplayEvidenceQualificationRequestV1<TQ>,
    error: CanonicalReplayEvidenceDecisionSubjectErrorV1,
}

impl<TQ> CanonicalReplayEvidenceDecisionSubjectFailureV1<TQ> {
    pub const fn error(&self) -> CanonicalReplayEvidenceDecisionSubjectErrorV1 {
        self.error
    }

    pub fn into_request(self) -> CanonicalReplayEvidenceQualificationRequestV1<TQ> {
        self.request
    }
}

fn fail<TQ>(
    request: CanonicalReplayEvidenceQualificationRequestV1<TQ>,
    error: CanonicalReplayEvidenceDecisionSubjectErrorV1,
) -> Result<
    CanonicalReplayEvidenceDecisionSubjectV1,
    CanonicalReplayEvidenceDecisionSubjectFailureV1<TQ>,
> {
    Err(CanonicalReplayEvidenceDecisionSubjectFailureV1 { request, error })
}

fn rederive_replay_basis(
    subject: CanonicalReplayEvidenceSubjectV1,
) -> Result<InitialReplayEvidenceBasisV1, CanonicalReplayEvidenceDecisionSubjectErrorV1> {
    let canonical = subject.latest_entry().manifest.subject.evidence.canonical_disposition();

    match canonical {
        CanonicalQualifiedEffectDispositionV1::Confirmed { .. } => {
            Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::ConfirmedEffectCannotReplay)
        }
        CanonicalQualifiedEffectDispositionV1::ProvenNotApplied { .. } => {
            Ok(InitialReplayEvidenceBasisV1::ProvenNotApplied)
        }
        CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
            recovery_policy, ..
        } => {
            let expected = recovery_policy_for(subject.stable_effect_identity().recovery_mode);
            if recovery_policy != expected {
                return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::RecoveryPolicyMismatch);
            }

            match recovery_policy {
                EffectRecoveryPolicyV1::ReconcileOnly => Err(
                    CanonicalReplayEvidenceDecisionSubjectErrorV1::TransactionalUnknownCannotReplay,
                ),
                EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted => {
                    Ok(InitialReplayEvidenceBasisV1::IdempotentOutcomeUnknown)
                }
                EffectRecoveryPolicyV1::NeverAutomaticRetry => Err(
                    CanonicalReplayEvidenceDecisionSubjectErrorV1::NonIdempotentUnknownCannotReplay,
                ),
            }
        }
    }
}

fn expected_terminal(
    basis: InitialReplayEvidenceBasisV1,
) -> Option<CanonicalCompletedOutcomeTerminalV1> {
    match basis {
        InitialReplayEvidenceBasisV1::ProvenNotApplied => {
            Some(CanonicalCompletedOutcomeTerminalV1::ProvenNotApplied)
        }
        InitialReplayEvidenceBasisV1::IdempotentOutcomeUnknown => None,
    }
}

fn validate_semantic_lineage(
    subject: CanonicalReplayEvidenceSubjectV1,
) -> Result<(), CanonicalReplayEvidenceDecisionSubjectErrorV1> {
    let invocation_record = subject.invocation_record();
    if !matches!(invocation_record, CanonicalCompletedInvocationRecordV1::Initial(_)) {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::SubjectNotInitialProvenance);
    }

    let history_head = subject.history_head();
    if history_head.invocation_record() != invocation_record {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::HistoryInvocationRecordMismatch);
    }

    let latest_entry = subject.latest_entry();
    if history_head.head() != Some(latest_entry.record) {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::HistoryHeadRecordMismatch);
    }

    let evidence = latest_entry.manifest.subject.evidence;
    if latest_entry.manifest.subject.invocation_record != invocation_record
        || evidence.invocation_record() != invocation_record
    {
        return Err(
            CanonicalReplayEvidenceDecisionSubjectErrorV1::LatestEvidenceInvocationRecordMismatch,
        );
    }

    let attempt = evidence.attempt();
    if attempt.attempt_id != subject.prior_attempt_id() {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::PriorAttemptMismatch);
    }
    if attempt.subject != subject.prior_effect_subject() {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::PriorEffectSubjectMismatch);
    }
    if attempt.subject.stable_identity() != subject.stable_effect_identity() {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::StableEffectIdentityMismatch);
    }

    let rederived_basis = rederive_replay_basis(subject)?;
    if rederived_basis != subject.basis() {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::ReplayBasisMismatch);
    }
    if history_head.terminal() != expected_terminal(rederived_basis) {
        return Err(CanonicalReplayEvidenceDecisionSubjectErrorV1::HistoryTerminalMismatch);
    }

    Ok(())
}

pub fn freeze_canonical_replay_evidence_decision_subject<TQ>(
    request: CanonicalReplayEvidenceQualificationRequestV1<TQ>,
) -> Result<
    CanonicalReplayEvidenceDecisionSubjectV1,
    CanonicalReplayEvidenceDecisionSubjectFailureV1<TQ>,
> {
    let subject = request.subject();
    let request_subject = request.request_subject();
    let qualification_time = request.qualification_time();
    let latest = qualification_time.latest_possible_unix_ms();

    if subject.schema_version() != SSF_SCHEMA_V1 {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::UnsupportedSubjectSchema,
        );
    }
    if subject.time_basis() != CanonicalReplayEvidenceSubjectTimeBasisV1::UnixMillisecondsUtc {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::UnsupportedSubjectTimeBasis,
        );
    }
    if let Err(error) = validate_semantic_lineage(subject) {
        return fail(request, error);
    }
    if request_subject.subject() != subject {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::RequestSubjectMismatch,
        );
    }
    if request_subject.qualification_time_receipt() != qualification_time.receipt_commitment() {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::QualificationTimeReceiptMismatch,
        );
    }
    if request_subject.qualification_latest_possible_unix_ms() != latest {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::QualificationTimeMismatch,
        );
    }

    let expected_valid_until = subject.valid_until().min(qualification_time.valid_until());
    if request_subject.valid_until() != expected_valid_until
        || request.valid_until() != expected_valid_until
    {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::NaturalExpiryMismatch,
        );
    }
    if expected_valid_until < latest {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::RequestAlreadyExpired,
        );
    }
    if request.contains_replay_authority()
        || request.contains_effect_authority()
        || request_subject.contains_replay_authority()
        || request_subject.contains_effect_authority()
        || request_subject.permits_third_attempt()
    {
        return fail(
            request,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::UnexpectedAuthority,
        );
    }

    Ok(CanonicalReplayEvidenceDecisionSubjectV1 {
        schema_version: SSF_SCHEMA_V1,
        domain: CanonicalReplayEvidenceDecisionDomainV1::EvidenceFitnessForAtMostOneReplay,
        request_subject,
        valid_until: expected_valid_until,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terminal_and_unknown_basis_have_distinct_expected_heads() {
        assert_eq!(
            expected_terminal(InitialReplayEvidenceBasisV1::ProvenNotApplied),
            Some(CanonicalCompletedOutcomeTerminalV1::ProvenNotApplied)
        );
        assert_eq!(
            expected_terminal(InitialReplayEvidenceBasisV1::IdempotentOutcomeUnknown),
            None
        );
    }

    #[test]
    fn semantic_mismatch_and_expiry_are_distinct_failures() {
        assert_ne!(
            CanonicalReplayEvidenceDecisionSubjectErrorV1::ReplayBasisMismatch,
            CanonicalReplayEvidenceDecisionSubjectErrorV1::RequestAlreadyExpired
        );
    }
}

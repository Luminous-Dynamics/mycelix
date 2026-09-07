// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical replay-policy authorization over freshly qualified canonical
//! outcome evidence.
//!
//! Evidence qualification and replay authority remain separate. This crate
//! consumes only `ReplayQualifiedCanonicalOutcomeEvidenceV1`, adds a newer
//! trusted policy-decision time, and asks an independently expected replay
//! policy generation whether one exact same-effect replay may be authorized.
//! It still creates no invocation attempt and performs no external effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::ActuatorStableEffectIdentityV1;
use mycelix_ssf_canonical_replay_evidence_qualification::{
    CanonicalReplayEvidenceBasisV1, CanonicalReplayEvidenceQualificationSubjectV1,
    ReplayEvidenceQualificationCommitment, ReplayEvidenceQualificationReceiptV1,
    ReplayQualifiedCanonicalOutcomeEvidenceV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_replay_authorization::{
    ExpectedReplayPolicyProfileV1, ReplayPolicyDescriptorV1, ReplayPolicyTimeBasisV1,
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

digest_type!(CanonicalReplayPolicyReceiptCommitment);
digest_type!(CanonicalReplayAuthorizationCommitment);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayPolicySubjectV1 {
    pub evidence_subject: CanonicalReplayEvidenceQualificationSubjectV1,
    pub evidence_receipt: ReplayEvidenceQualificationReceiptV1,
    pub evidence_qualification: ReplayEvidenceQualificationCommitment,
    pub evidence_valid_until: u64,
    pub stable_effect_identity: ActuatorStableEffectIdentityV1,
    pub basis: CanonicalReplayEvidenceBasisV1,
    pub policy_time_receipt: CurrentTimeReceiptCommitment,
    pub policy_latest_possible_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReplayPolicyDispositionV1 {
    AuthorizeExactSameEffect {
        authorization: CanonicalReplayAuthorizationCommitment,
    },
    Reject,
    Defer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayPolicyReceiptV1 {
    pub schema_version: u16,
    pub policy: ReplayPolicyDescriptorV1,
    pub subject: CanonicalReplayPolicySubjectV1,
    pub disposition: CanonicalReplayPolicyDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: CanonicalReplayPolicyReceiptCommitment,
}

pub trait CanonicalReplayPolicyEvaluatorV1 {
    type Error;

    fn descriptor(&self) -> ReplayPolicyDescriptorV1;

    fn evaluate_canonical_replay(
        &self,
        subject: &CanonicalReplayPolicySubjectV1,
    ) -> Result<CanonicalReplayPolicyReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReplayPolicyPreparationErrorV1 {
    PolicyTimeRegressedBeforeEvidenceQualification,
    EvidenceQualificationAlreadyExpired,
    PolicyTimeAlreadyExpired,
}

pub struct CanonicalReplayPolicyPreparationFailureV1<Q, S, HTQ, QTQ, PTQ> {
    evidence: ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ>,
    policy_time: QualifiedCurrentTimeV1<PTQ>,
    error: CanonicalReplayPolicyPreparationErrorV1,
}

impl<Q, S, HTQ, QTQ, PTQ> CanonicalReplayPolicyPreparationFailureV1<Q, S, HTQ, QTQ, PTQ> {
    pub const fn error(&self) -> CanonicalReplayPolicyPreparationErrorV1 {
        self.error
    }

    pub fn into_parts(
        self,
    ) -> (
        ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ>,
        QualifiedCurrentTimeV1<PTQ>,
    ) {
        (self.evidence, self.policy_time)
    }
}

pub struct CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ> {
    evidence: ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ>,
    policy_time: QualifiedCurrentTimeV1<PTQ>,
    subject: CanonicalReplayPolicySubjectV1,
}

impl<Q, S, HTQ, QTQ, PTQ> CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ> {
    pub const fn evidence(&self) -> &ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ> {
        &self.evidence
    }

    pub const fn policy_time(&self) -> &QualifiedCurrentTimeV1<PTQ> {
        &self.policy_time
    }

    pub const fn subject(&self) -> CanonicalReplayPolicySubjectV1 {
        self.subject
    }

    pub const fn contains_replay_authority(&self) -> bool {
        false
    }
}

pub fn prepare_canonical_replay_policy_candidate<Q, S, HTQ, QTQ, PTQ>(
    evidence: ReplayQualifiedCanonicalOutcomeEvidenceV1<Q, S, HTQ, QTQ>,
    policy_time: QualifiedCurrentTimeV1<PTQ>,
) -> Result<
    CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ>,
    CanonicalReplayPolicyPreparationFailureV1<Q, S, HTQ, QTQ, PTQ>,
> {
    let latest = policy_time.latest_possible_unix_ms();
    let evidence_subject = evidence.subject();

    let error = if latest < evidence_subject.qualification_latest_possible_unix_ms {
        Some(CanonicalReplayPolicyPreparationErrorV1::PolicyTimeRegressedBeforeEvidenceQualification)
    } else if evidence.valid_until() < latest {
        Some(CanonicalReplayPolicyPreparationErrorV1::EvidenceQualificationAlreadyExpired)
    } else if policy_time.valid_until() < latest {
        Some(CanonicalReplayPolicyPreparationErrorV1::PolicyTimeAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(CanonicalReplayPolicyPreparationFailureV1 {
            evidence,
            policy_time,
            error,
        });
    }

    let subject = CanonicalReplayPolicySubjectV1 {
        evidence_subject,
        evidence_receipt: evidence.receipt(),
        evidence_qualification: evidence.qualification(),
        evidence_valid_until: evidence.valid_until(),
        stable_effect_identity: evidence_subject.stable_effect_identity,
        basis: evidence_subject.basis,
        policy_time_receipt: policy_time.receipt_commitment(),
        policy_latest_possible_unix_ms: latest,
    };

    Ok(CanonicalReplayPolicyCandidateV1 {
        evidence,
        policy_time,
        subject,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayAuthorizationFailureReasonV1 {
    UnsupportedPolicyTimeBasis,
    PolicyAlreadyExpired,
    PolicyDescriptorMismatchBefore,
    PolicyDescriptorChangedAfter,
    PolicyEvaluationError,
    UnsupportedReceiptSchema,
    ReceiptPolicyMismatch,
    ReceiptSubjectMismatch,
    ReceiptOutlivesPolicy,
    ReceiptOutlivesEvidenceQualification,
    ReceiptOutlivesPolicyTime,
    ReceiptAlreadyExpired,
}

pub struct CanonicalReplayAuthorizationFailureV1<P, Q, S, HTQ, QTQ, PTQ> {
    candidate: CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ>,
    expected_policy: ExpectedReplayPolicyProfileV1,
    reason: CanonicalReplayAuthorizationFailureReasonV1,
    _policy: PhantomData<fn() -> P>,
}

impl<P, Q, S, HTQ, QTQ, PTQ>
    CanonicalReplayAuthorizationFailureV1<P, Q, S, HTQ, QTQ, PTQ>
{
    pub const fn reason(&self) -> CanonicalReplayAuthorizationFailureReasonV1 {
        self.reason
    }

    pub fn into_candidate(self) -> CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ> {
        self.candidate
    }

    pub const fn expected_policy(&self) -> ExpectedReplayPolicyProfileV1 {
        self.expected_policy
    }
}

pub struct AuthorizedCanonicalReplayV1<P, Q, S, HTQ, QTQ, PTQ> {
    candidate: CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ>,
    expected_policy: ExpectedReplayPolicyProfileV1,
    receipt: CanonicalReplayPolicyReceiptV1,
    authorization: CanonicalReplayAuthorizationCommitment,
    valid_until: u64,
    _policy: PhantomData<fn() -> P>,
}

impl<P, Q, S, HTQ, QTQ, PTQ> AuthorizedCanonicalReplayV1<P, Q, S, HTQ, QTQ, PTQ> {
    pub const fn subject(&self) -> CanonicalReplayPolicySubjectV1 {
        self.candidate.subject
    }

    pub const fn stable_effect_identity(&self) -> ActuatorStableEffectIdentityV1 {
        self.candidate.subject.stable_effect_identity
    }

    pub const fn receipt(&self) -> CanonicalReplayPolicyReceiptV1 {
        self.receipt
    }

    pub const fn authorization(&self) -> CanonicalReplayAuthorizationCommitment {
        self.authorization
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn expected_policy(&self) -> ExpectedReplayPolicyProfileV1 {
        self.expected_policy
    }

    pub const fn authorizes_at_most_one_new_attempt(&self) -> bool {
        true
    }

    pub const fn requires_fresh_pre_invocation_qualification(&self) -> bool {
        true
    }

    pub const fn requires_durable_replay_journal(&self) -> bool {
        true
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub struct RejectedCanonicalReplayV1<Q, S, HTQ, QTQ, PTQ> {
    candidate: CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ>,
    receipt: CanonicalReplayPolicyReceiptV1,
}

impl<Q, S, HTQ, QTQ, PTQ> RejectedCanonicalReplayV1<Q, S, HTQ, QTQ, PTQ> {
    pub const fn receipt(&self) -> CanonicalReplayPolicyReceiptV1 {
        self.receipt
    }

    pub const fn replay_allowed(&self) -> bool {
        false
    }

    pub fn into_candidate(self) -> CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ> {
        self.candidate
    }
}

pub struct DeferredCanonicalReplayV1<Q, S, HTQ, QTQ, PTQ> {
    candidate: CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ>,
    receipt: CanonicalReplayPolicyReceiptV1,
}

impl<Q, S, HTQ, QTQ, PTQ> DeferredCanonicalReplayV1<Q, S, HTQ, QTQ, PTQ> {
    pub const fn receipt(&self) -> CanonicalReplayPolicyReceiptV1 {
        self.receipt
    }

    pub const fn replay_allowed(&self) -> bool {
        false
    }

    pub fn into_candidate(self) -> CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ> {
        self.candidate
    }
}

pub enum CanonicalReplayAuthorizationDecisionV1<P, Q, S, HTQ, QTQ, PTQ> {
    Authorized(AuthorizedCanonicalReplayV1<P, Q, S, HTQ, QTQ, PTQ>),
    Rejected(RejectedCanonicalReplayV1<Q, S, HTQ, QTQ, PTQ>),
    Deferred(DeferredCanonicalReplayV1<Q, S, HTQ, QTQ, PTQ>),
}

pub fn evaluate_canonical_replay_authorization<P, Q, S, HTQ, QTQ, PTQ>(
    candidate: CanonicalReplayPolicyCandidateV1<Q, S, HTQ, QTQ, PTQ>,
    expected_policy: ExpectedReplayPolicyProfileV1,
    policy: &P,
) -> Result<
    CanonicalReplayAuthorizationDecisionV1<P, Q, S, HTQ, QTQ, PTQ>,
    CanonicalReplayAuthorizationFailureV1<P, Q, S, HTQ, QTQ, PTQ>,
>
where
    P: CanonicalReplayPolicyEvaluatorV1,
{
    let latest = candidate.policy_time.latest_possible_unix_ms();

    let failure = |candidate,
                   expected_policy,
                   reason| CanonicalReplayAuthorizationFailureV1 {
        candidate,
        expected_policy,
        reason,
        _policy: PhantomData,
    };

    if expected_policy.descriptor().time_basis != ReplayPolicyTimeBasisV1::UnixMillisecondsUtc {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::UnsupportedPolicyTimeBasis,
        ));
    }
    if expected_policy.descriptor().valid_until < latest {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::PolicyAlreadyExpired,
        ));
    }
    if policy.descriptor() != expected_policy.descriptor() {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::PolicyDescriptorMismatchBefore,
        ));
    }

    let receipt = match policy.evaluate_canonical_replay(&candidate.subject) {
        Ok(value) => value,
        Err(_) => {
            return Err(failure(
                candidate,
                expected_policy,
                CanonicalReplayAuthorizationFailureReasonV1::PolicyEvaluationError,
            ));
        }
    };
    if policy.descriptor() != expected_policy.descriptor() {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::PolicyDescriptorChangedAfter,
        ));
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::UnsupportedReceiptSchema,
        ));
    }
    if receipt.policy != expected_policy.descriptor() {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::ReceiptPolicyMismatch,
        ));
    }
    if receipt.subject != candidate.subject {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::ReceiptSubjectMismatch,
        ));
    }
    if receipt.valid_until > expected_policy.descriptor().valid_until {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::ReceiptOutlivesPolicy,
        ));
    }
    if receipt.valid_until > candidate.evidence.valid_until() {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::ReceiptOutlivesEvidenceQualification,
        ));
    }
    if receipt.valid_until > candidate.policy_time.valid_until() {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::ReceiptOutlivesPolicyTime,
        ));
    }
    if receipt.valid_until < latest {
        return Err(failure(
            candidate,
            expected_policy,
            CanonicalReplayAuthorizationFailureReasonV1::ReceiptAlreadyExpired,
        ));
    }

    match receipt.disposition {
        CanonicalReplayPolicyDispositionV1::AuthorizeExactSameEffect { authorization } => {
            let valid_until = receipt
                .valid_until
                .min(candidate.evidence.valid_until())
                .min(candidate.policy_time.valid_until())
                .min(expected_policy.descriptor().valid_until);
            Ok(CanonicalReplayAuthorizationDecisionV1::Authorized(
                AuthorizedCanonicalReplayV1 {
                    candidate,
                    expected_policy,
                    receipt,
                    authorization,
                    valid_until,
                    _policy: PhantomData,
                },
            ))
        }
        CanonicalReplayPolicyDispositionV1::Reject => Ok(
            CanonicalReplayAuthorizationDecisionV1::Rejected(RejectedCanonicalReplayV1 {
                candidate,
                receipt,
            }),
        ),
        CanonicalReplayPolicyDispositionV1::Defer => Ok(
            CanonicalReplayAuthorizationDecisionV1::Deferred(DeferredCanonicalReplayV1 {
                candidate,
                receipt,
            }),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn authorization_is_explicitly_single_attempt() {
        assert_ne!(
            core::mem::size_of::<CanonicalReplayAuthorizationCommitment>(),
            0
        );
    }

    #[test]
    fn evidence_and_policy_are_distinct_commitment_domains() {
        assert_eq!(
            core::mem::size_of::<ReplayEvidenceQualificationCommitment>(),
            core::mem::size_of::<CanonicalReplayAuthorizationCommitment>()
        );
    }
}
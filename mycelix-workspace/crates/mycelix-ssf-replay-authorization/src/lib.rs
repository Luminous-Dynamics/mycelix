// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit replay authorization for SSF actuator attempts.
//!
//! Replay is authority-bearing. Durable uncertainty, actuator idempotency, or
//! a recovery-mode flag never creates replay authority by itself. Replay
//! eligibility consumes an exact durable outcome-head record/manifest pairing;
//! callers cannot bypass that evidence boundary by supplying a raw compatible
//! history head and manifest.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    ActuatorEffectSubjectV1, ActuatorInvocationAttemptId, ActuatorStableEffectIdentityV1,
    EffectRecoveryPolicyV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_durable_effect_outcome_history::{
    EffectOutcomeHistoryHeadReceiptCommitment, EffectOutcomeHistoryHeadV1,
    EffectOutcomeObservationManifestV1, EffectOutcomeStateV1,
    QualifiedEffectOutcomeHistoryHeadV1,
};
use mycelix_ssf_effect_outcome_head_entry::{
    EffectOutcomeHeadEntryReceiptCommitment, QualifiedExactEffectOutcomeHeadEntryV1,
};
use mycelix_ssf_pre_invocation_actuator_qualification::ActuatorRecoveryModeV1;

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

digest_type!(ReplayPolicyIdentityCommitment);
digest_type!(ReplayPolicyCommitment);
digest_type!(ReplayPolicyReceiptCommitment);
digest_type!(ReplayAuthorizationCommitment);

generation_type!(ReplayPolicyGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplayPolicyTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ReplayPolicyDescriptorV1 {
    pub stable_identity: ReplayPolicyIdentityCommitment,
    pub policy: ReplayPolicyCommitment,
    pub generation: ReplayPolicyGeneration,
    pub time_basis: ReplayPolicyTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedReplayPolicyProfileV1 {
    descriptor: ReplayPolicyDescriptorV1,
}

impl ExpectedReplayPolicyProfileV1 {
    pub const fn from_trusted_configuration(descriptor: ReplayPolicyDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> ReplayPolicyDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplayBasisV1 {
    ProvenNotApplied,
    IdempotentOutcomeUnknown,
}

/// Exact immutable subject evaluated by local replay policy.
///
/// `prior_effect_subject` retains the previous attempt's fresh qualification
/// evidence. `stable_effect_identity` defines the semantic/physical effect a
/// future freshly qualified replay must preserve. `head_entry_receipt` proves
/// that `latest_outcome` is explicitly paired with the exact durable head
/// record rather than merely being generation-compatible metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayAuthorizationSubjectV1 {
    pub history_head: EffectOutcomeHistoryHeadV1,
    pub history_head_receipt: EffectOutcomeHistoryHeadReceiptCommitment,
    pub head_entry_receipt: EffectOutcomeHeadEntryReceiptCommitment,
    pub latest_outcome: EffectOutcomeObservationManifestV1,
    pub prior_attempt_id: ActuatorInvocationAttemptId,
    pub prior_effect_subject: ActuatorEffectSubjectV1,
    pub stable_effect_identity: ActuatorStableEffectIdentityV1,
    pub basis: ReplayBasisV1,
    pub decision_time_receipt: CurrentTimeReceiptCommitment,
    pub decision_latest_possible_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayPolicyDispositionV1 {
    AuthorizeExactSameEffect {
        authorization: ReplayAuthorizationCommitment,
    },
    Reject,
    Defer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayPolicyReceiptV1 {
    pub schema_version: u16,
    pub policy: ReplayPolicyDescriptorV1,
    pub subject: ReplayAuthorizationSubjectV1,
    pub disposition: ReplayPolicyDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: ReplayPolicyReceiptCommitment,
}

pub trait ReplayPolicyEvaluatorV1 {
    type Error;

    fn descriptor(&self) -> ReplayPolicyDescriptorV1;

    fn evaluate_replay(
        &self,
        subject: &ReplayAuthorizationSubjectV1,
    ) -> Result<ReplayPolicyReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayEligibilityErrorV1 {
    ConfirmedEffectCannotReplay,
    TransactionalUnknownRequiresReconciliation,
    NonIdempotentUnknownCannotReplay,
    UnknownRecoveryPolicyMismatch,
}

fn expected_recovery_policy(mode: ActuatorRecoveryModeV1) -> EffectRecoveryPolicyV1 {
    match mode {
        ActuatorRecoveryModeV1::TransactionalClaimKey => EffectRecoveryPolicyV1::ReconcileOnly,
        ActuatorRecoveryModeV1::IdempotentClaimKey => {
            EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted
        }
        ActuatorRecoveryModeV1::NonIdempotentNoAutomaticRetry => {
            EffectRecoveryPolicyV1::NeverAutomaticRetry
        }
    }
}

fn basis_from_state(state: EffectOutcomeStateV1) -> Result<ReplayBasisV1, ReplayEligibilityErrorV1> {
    match state {
        EffectOutcomeStateV1::Confirmed { .. } => {
            Err(ReplayEligibilityErrorV1::ConfirmedEffectCannotReplay)
        }
        EffectOutcomeStateV1::ProvenNotApplied { .. } => Ok(ReplayBasisV1::ProvenNotApplied),
        EffectOutcomeStateV1::OutcomeUnknown {
            recovery_policy, ..
        } => match recovery_policy {
            EffectRecoveryPolicyV1::ReconcileOnly => {
                Err(ReplayEligibilityErrorV1::TransactionalUnknownRequiresReconciliation)
            }
            EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted => {
                Ok(ReplayBasisV1::IdempotentOutcomeUnknown)
            }
            EffectRecoveryPolicyV1::NeverAutomaticRetry => {
                Err(ReplayEligibilityErrorV1::NonIdempotentUnknownCannotReplay)
            }
        },
    }
}

pub struct ReplayEligibilityFailureV1<S, TQ> {
    exact_entry: QualifiedExactEffectOutcomeHeadEntryV1<S, TQ>,
    error: ReplayEligibilityErrorV1,
}

impl<S, TQ> ReplayEligibilityFailureV1<S, TQ> {
    pub const fn error(&self) -> ReplayEligibilityErrorV1 {
        self.error
    }

    pub fn into_exact_entry(self) -> QualifiedExactEffectOutcomeHeadEntryV1<S, TQ> {
        self.exact_entry
    }
}

pub struct ReplayEligibleEvidenceV1<S, TQ> {
    exact_entry: QualifiedExactEffectOutcomeHeadEntryV1<S, TQ>,
    subject: ReplayAuthorizationSubjectV1,
}

impl<S, TQ> ReplayEligibleEvidenceV1<S, TQ> {
    pub const fn exact_entry(&self) -> &QualifiedExactEffectOutcomeHeadEntryV1<S, TQ> {
        &self.exact_entry
    }

    pub const fn history(&self) -> &QualifiedEffectOutcomeHistoryHeadV1<S> {
        self.exact_entry.qualified_head()
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        self.exact_entry.current_time()
    }

    pub const fn subject(&self) -> ReplayAuthorizationSubjectV1 {
        self.subject
    }
}

/// Establish structural replay eligibility from an exact durable head-entry
/// pairing. There is intentionally no public overload accepting a raw history
/// head and current-time token.
pub fn qualify_replay_eligibility<S, TQ>(
    exact_entry: QualifiedExactEffectOutcomeHeadEntryV1<S, TQ>,
) -> Result<ReplayEligibleEvidenceV1<S, TQ>, ReplayEligibilityFailureV1<S, TQ>> {
    let head_read = exact_entry.qualified_head().receipt();
    let entry = exact_entry.entry_receipt();
    let latest = exact_entry.manifest();
    let prior_effect_subject = latest.subject.journal_manifest.attempt.subject;

    if let EffectOutcomeStateV1::OutcomeUnknown {
        recovery_policy, ..
    } = latest.subject.state
    {
        if recovery_policy != expected_recovery_policy(prior_effect_subject.recovery_mode) {
            return Err(ReplayEligibilityFailureV1 {
                exact_entry,
                error: ReplayEligibilityErrorV1::UnknownRecoveryPolicyMismatch,
            });
        }
    }

    let basis = match basis_from_state(latest.subject.state) {
        Ok(basis) => basis,
        Err(error) => return Err(ReplayEligibilityFailureV1 { exact_entry, error }),
    };

    let subject = ReplayAuthorizationSubjectV1 {
        history_head: entry.head,
        history_head_receipt: head_read.receipt_commitment,
        head_entry_receipt: entry.receipt_commitment,
        latest_outcome: latest,
        prior_attempt_id: latest.subject.journal_manifest.attempt.attempt_id,
        prior_effect_subject,
        stable_effect_identity: prior_effect_subject.stable_identity(),
        basis,
        decision_time_receipt: exact_entry.current_time().receipt_commitment(),
        decision_latest_possible_unix_ms: exact_entry.current_time().latest_possible_unix_ms(),
    };

    Ok(ReplayEligibleEvidenceV1 {
        exact_entry,
        subject,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplayAuthorizationFailureReasonV1 {
    UnsupportedPolicyTimeBasis,
    PolicyAlreadyExpired,
    PolicyDescriptorMismatchBefore,
    PolicyDescriptorChangedAfter,
    PolicyEvaluationError,
    UnsupportedReceiptSchema,
    ReceiptPolicyMismatch,
    ReceiptSubjectMismatch,
    ReceiptOutlivesPolicy,
    ReceiptOutlivesEvidence,
    ReceiptAlreadyExpired,
}

pub struct ReplayAuthorizationFailureV1<P, S, TQ> {
    eligible: ReplayEligibleEvidenceV1<S, TQ>,
    expected_policy: ExpectedReplayPolicyProfileV1,
    reason: ReplayAuthorizationFailureReasonV1,
    _policy: PhantomData<fn() -> P>,
}

impl<P, S, TQ> ReplayAuthorizationFailureV1<P, S, TQ> {
    pub const fn reason(&self) -> ReplayAuthorizationFailureReasonV1 {
        self.reason
    }

    pub fn into_eligible(self) -> ReplayEligibleEvidenceV1<S, TQ> {
        self.eligible
    }

    pub const fn expected_policy(&self) -> ExpectedReplayPolicyProfileV1 {
        self.expected_policy
    }
}

pub struct AuthorizedReplayV1<P, S, TQ> {
    eligible: ReplayEligibleEvidenceV1<S, TQ>,
    expected_policy: ExpectedReplayPolicyProfileV1,
    receipt: ReplayPolicyReceiptV1,
    authorization: ReplayAuthorizationCommitment,
    valid_until: u64,
    _policy: PhantomData<fn() -> P>,
}

impl<P, S, TQ> AuthorizedReplayV1<P, S, TQ> {
    pub const fn subject(&self) -> ReplayAuthorizationSubjectV1 {
        self.eligible.subject
    }

    pub const fn stable_effect_identity(&self) -> ActuatorStableEffectIdentityV1 {
        self.eligible.subject.stable_effect_identity
    }

    pub const fn receipt(&self) -> ReplayPolicyReceiptV1 {
        self.receipt
    }

    pub const fn authorization(&self) -> ReplayAuthorizationCommitment {
        self.authorization
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn authorizes_at_most_one_new_attempt(&self) -> bool {
        true
    }

    pub const fn requires_fresh_pre_invocation_qualification(&self) -> bool {
        true
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn expected_policy(&self) -> ExpectedReplayPolicyProfileV1 {
        self.expected_policy
    }
}

pub struct RejectedReplayV1<S, TQ> {
    eligible: ReplayEligibleEvidenceV1<S, TQ>,
    receipt: ReplayPolicyReceiptV1,
}

impl<S, TQ> RejectedReplayV1<S, TQ> {
    pub const fn receipt(&self) -> ReplayPolicyReceiptV1 {
        self.receipt
    }

    pub const fn replay_allowed(&self) -> bool {
        false
    }

    pub fn into_eligible(self) -> ReplayEligibleEvidenceV1<S, TQ> {
        self.eligible
    }
}

pub struct DeferredReplayV1<S, TQ> {
    eligible: ReplayEligibleEvidenceV1<S, TQ>,
    receipt: ReplayPolicyReceiptV1,
}

impl<S, TQ> DeferredReplayV1<S, TQ> {
    pub const fn receipt(&self) -> ReplayPolicyReceiptV1 {
        self.receipt
    }

    pub const fn replay_allowed(&self) -> bool {
        false
    }

    pub fn into_eligible(self) -> ReplayEligibleEvidenceV1<S, TQ> {
        self.eligible
    }
}

pub enum ReplayAuthorizationDecisionV1<P, S, TQ> {
    Authorized(AuthorizedReplayV1<P, S, TQ>),
    Rejected(RejectedReplayV1<S, TQ>),
    Deferred(DeferredReplayV1<S, TQ>),
}

pub fn evaluate_replay_authorization<P, S, TQ>(
    eligible: ReplayEligibleEvidenceV1<S, TQ>,
    expected_policy: ExpectedReplayPolicyProfileV1,
    policy: &P,
) -> Result<ReplayAuthorizationDecisionV1<P, S, TQ>, ReplayAuthorizationFailureV1<P, S, TQ>>
where
    P: ReplayPolicyEvaluatorV1,
{
    let latest_now = eligible.current_time().latest_possible_unix_ms();
    let evidence_valid_until = eligible.exact_entry.valid_until();

    if expected_policy.descriptor.time_basis != ReplayPolicyTimeBasisV1::UnixMillisecondsUtc {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::UnsupportedPolicyTimeBasis,
            _policy: PhantomData,
        });
    }
    if expected_policy.descriptor.valid_until < latest_now {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::PolicyAlreadyExpired,
            _policy: PhantomData,
        });
    }
    if policy.descriptor() != expected_policy.descriptor {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::PolicyDescriptorMismatchBefore,
            _policy: PhantomData,
        });
    }

    let receipt = match policy.evaluate_replay(&eligible.subject) {
        Ok(receipt) => receipt,
        Err(_) => {
            return Err(ReplayAuthorizationFailureV1 {
                eligible,
                expected_policy,
                reason: ReplayAuthorizationFailureReasonV1::PolicyEvaluationError,
                _policy: PhantomData,
            });
        }
    };

    if policy.descriptor() != expected_policy.descriptor {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::PolicyDescriptorChangedAfter,
            _policy: PhantomData,
        });
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::UnsupportedReceiptSchema,
            _policy: PhantomData,
        });
    }
    if receipt.policy != expected_policy.descriptor {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::ReceiptPolicyMismatch,
            _policy: PhantomData,
        });
    }
    if receipt.subject != eligible.subject {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::ReceiptSubjectMismatch,
            _policy: PhantomData,
        });
    }
    if receipt.valid_until > expected_policy.descriptor.valid_until {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::ReceiptOutlivesPolicy,
            _policy: PhantomData,
        });
    }
    if receipt.valid_until > evidence_valid_until {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::ReceiptOutlivesEvidence,
            _policy: PhantomData,
        });
    }
    if receipt.valid_until < latest_now {
        return Err(ReplayAuthorizationFailureV1 {
            eligible,
            expected_policy,
            reason: ReplayAuthorizationFailureReasonV1::ReceiptAlreadyExpired,
            _policy: PhantomData,
        });
    }

    match receipt.disposition {
        ReplayPolicyDispositionV1::AuthorizeExactSameEffect { authorization } => {
            let valid_until = evidence_valid_until
                .min(expected_policy.descriptor.valid_until)
                .min(receipt.valid_until);
            Ok(ReplayAuthorizationDecisionV1::Authorized(AuthorizedReplayV1 {
                eligible,
                expected_policy,
                receipt,
                authorization,
                valid_until,
                _policy: PhantomData,
            }))
        }
        ReplayPolicyDispositionV1::Reject => {
            Ok(ReplayAuthorizationDecisionV1::Rejected(RejectedReplayV1 {
                eligible,
                receipt,
            }))
        }
        ReplayPolicyDispositionV1::Defer => {
            Ok(ReplayAuthorizationDecisionV1::Deferred(DeferredReplayV1 {
                eligible,
                receipt,
            }))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_actuator_effect_protocol::{
        ActuatorEffectAmbiguityReasonV1, ActuatorEffectOutcomeCommitment,
        ActuatorEffectReceiptCommitment, ActuatorNonApplicationEvidenceCommitment,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    #[test]
    fn confirmed_is_a_structural_replay_stop() {
        let state = EffectOutcomeStateV1::Confirmed {
            outcome: ActuatorEffectOutcomeCommitment::from_bytes(bytes(1)),
            actuator_receipt: ActuatorEffectReceiptCommitment::from_bytes(bytes(2)),
        };
        assert_eq!(
            basis_from_state(state),
            Err(ReplayEligibilityErrorV1::ConfirmedEffectCannotReplay)
        );
    }

    #[test]
    fn proven_non_application_can_become_policy_candidate() {
        let state = EffectOutcomeStateV1::ProvenNotApplied {
            evidence: ActuatorNonApplicationEvidenceCommitment::from_bytes(bytes(1)),
            actuator_receipt: ActuatorEffectReceiptCommitment::from_bytes(bytes(2)),
        };
        assert_eq!(basis_from_state(state), Ok(ReplayBasisV1::ProvenNotApplied));
    }

    #[test]
    fn transactional_unknown_is_reconcile_only() {
        let state = EffectOutcomeStateV1::OutcomeUnknown {
            reason: ActuatorEffectAmbiguityReasonV1::ActuatorErrorAfterInvocation,
            actuator_receipt: None,
            recovery_policy: EffectRecoveryPolicyV1::ReconcileOnly,
        };
        assert_eq!(
            basis_from_state(state),
            Err(ReplayEligibilityErrorV1::TransactionalUnknownRequiresReconciliation)
        );
    }

    #[test]
    fn idempotent_unknown_is_only_a_candidate() {
        let state = EffectOutcomeStateV1::OutcomeUnknown {
            reason: ActuatorEffectAmbiguityReasonV1::ActuatorErrorAfterInvocation,
            actuator_receipt: None,
            recovery_policy: EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted,
        };
        assert_eq!(
            basis_from_state(state),
            Ok(ReplayBasisV1::IdempotentOutcomeUnknown)
        );
    }

    #[test]
    fn non_idempotent_unknown_cannot_replay() {
        let state = EffectOutcomeStateV1::OutcomeUnknown {
            reason: ActuatorEffectAmbiguityReasonV1::ActuatorErrorAfterInvocation,
            actuator_receipt: None,
            recovery_policy: EffectRecoveryPolicyV1::NeverAutomaticRetry,
        };
        assert_eq!(
            basis_from_state(state),
            Err(ReplayEligibilityErrorV1::NonIdempotentUnknownCannotReplay)
        );
    }
}

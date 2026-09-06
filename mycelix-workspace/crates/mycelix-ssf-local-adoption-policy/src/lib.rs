// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Local adoption policy qualification for the Mycelix Solar-System Federation
//! profile.
//!
//! This layer consumes exact `LocalAdoptionReadyEvidenceV1` and asks one
//! independently approved local policy evaluator to produce an explicit
//! Approve / Reject / Defer decision bound to the entire exact evidence subject.
//!
//! Approval is still not state-install authority and not effect authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::{array, marker::PhantomData};

use mycelix_ssf_authorization::{AuthorizationReceiptCommitment, VerifierDescriptorV1};
use mycelix_ssf_contracts::{
    ConsequenceClass, EvidenceCommitment, PolicyDigest, ValidityInputsV1, SSF_SCHEMA_V1,
};
use mycelix_ssf_local_adoption_readiness::{
    LocalAdoptionReadyEvidenceV1, LocalPrerequisiteEvidenceV1,
    LocalQualificationReceiptCommitment,
};
use mycelix_ssf_reconciliation::CandidateHistoryStepV1;
use mycelix_ssf_snapshots::FederationStateHeadV1;

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

digest_type!(PolicyEvaluatorIdentityCommitment);
digest_type!(PolicyEvaluationReceiptCommitment);
digest_type!(LocalAdoptionScopeCommitment);
generation_type!(PolicyEvaluatorGeneration);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LocalAdoptionPolicyDescriptorV1 {
    pub identity: PolicyEvaluatorIdentityCommitment,
    pub policy_digest: PolicyDigest,
    pub generation: PolicyEvaluatorGeneration,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedLocalAdoptionPolicyProfileV1 {
    descriptor: LocalAdoptionPolicyDescriptorV1,
}

impl ExpectedLocalAdoptionPolicyProfileV1 {
    pub const fn from_trusted_configuration(descriptor: LocalAdoptionPolicyDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> LocalAdoptionPolicyDescriptorV1 {
        self.descriptor
    }
}

/// Closed-world local policy decision. Reject and Defer are successful policy
/// evaluations, not verifier errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LocalAdoptionDecisionV1 {
    Approve,
    Reject,
    Defer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolicySubjectRemoteStepV1 {
    pub candidate: CandidateHistoryStepV1,
    pub verifier: VerifierDescriptorV1,
    pub authorization_receipt: AuthorizationReceiptCommitment,
    pub valid_until: u64,
}

/// Exact immutable subject presented to local policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocalAdoptionPolicySubjectV1<const R: usize> {
    pub common_ancestor: FederationStateHeadV1,
    pub common_ancestor_qualifier: VerifierDescriptorV1,
    pub common_ancestor_evidence_root: EvidenceCommitment,
    pub common_ancestor_receipt: LocalQualificationReceiptCommitment,
    pub local_state: FederationStateHeadV1,
    pub local_context_qualifier: VerifierDescriptorV1,
    pub local_prerequisite_evidence: LocalPrerequisiteEvidenceV1,
    pub local_prerequisite_validity: ValidityInputsV1,
    pub local_context_receipt: LocalQualificationReceiptCommitment,
    pub remote_target: FederationStateHeadV1,
    pub readiness_valid_until: u64,
    pub remote_steps: [PolicySubjectRemoteStepV1; R],
}

fn subject_from_readiness<EV, RV, AQ, LQ, const L: usize, const R: usize>(
    readiness: &LocalAdoptionReadyEvidenceV1<EV, RV, AQ, LQ, L, R>,
) -> LocalAdoptionPolicySubjectV1<R> {
    let assessment = readiness.coverage().safety_floor().assessment();
    let remote_steps = array::from_fn(|index| {
        let step = &readiness.coverage().remote_path()[index];
        PolicySubjectRemoteStepV1 {
            candidate: step.candidate(),
            verifier: step.verifier(),
            authorization_receipt: step.receipt_commitment(),
            valid_until: step.valid_until(),
        }
    });

    LocalAdoptionPolicySubjectV1 {
        common_ancestor: readiness.common_ancestor().ancestor(),
        common_ancestor_qualifier: readiness.common_ancestor().expected_profile().descriptor(),
        common_ancestor_evidence_root: readiness.common_ancestor().evidence_root(),
        common_ancestor_receipt: readiness.common_ancestor().receipt_commitment(),
        local_state: readiness.current_local_context().local_state(),
        local_context_qualifier: readiness
            .current_local_context()
            .expected_profile()
            .descriptor(),
        local_prerequisite_evidence: readiness.current_local_context().prerequisite_evidence(),
        local_prerequisite_validity: readiness.current_local_context().prerequisite_validity(),
        local_context_receipt: readiness.current_local_context().receipt_commitment(),
        remote_target: assessment.remote_tip(),
        readiness_valid_until: readiness.valid_until(),
        remote_steps,
    }
}

/// Untrusted result returned by a local policy evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocalAdoptionPolicyReceiptV1<const R: usize> {
    pub schema_version: u16,
    pub evaluator: LocalAdoptionPolicyDescriptorV1,
    pub subject: LocalAdoptionPolicySubjectV1<R>,
    pub decision: LocalAdoptionDecisionV1,
    pub adoption_scope: LocalAdoptionScopeCommitment,
    /// Upper bound on any downstream authority that may ever be derived from
    /// this policy decision. It is not itself effect authority.
    pub maximum_follow_on_consequence: ConsequenceClass,
    /// Policy-defined diagnostic/reason code with no authority semantics.
    pub reason_code: u16,
    pub valid_until: u64,
    pub receipt_commitment: PolicyEvaluationReceiptCommitment,
}

impl<const R: usize> LocalAdoptionPolicyReceiptV1<R> {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        evaluator: LocalAdoptionPolicyDescriptorV1,
        subject: LocalAdoptionPolicySubjectV1<R>,
        decision: LocalAdoptionDecisionV1,
        adoption_scope: LocalAdoptionScopeCommitment,
        maximum_follow_on_consequence: ConsequenceClass,
        reason_code: u16,
        valid_until: u64,
        receipt_commitment: PolicyEvaluationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            evaluator,
            subject,
            decision,
            adoption_scope,
            maximum_follow_on_consequence,
            reason_code,
            valid_until,
            receipt_commitment,
        }
    }
}

pub trait LocalAdoptionPolicyEvaluatorV1<const R: usize> {
    type Error;

    fn descriptor(&self) -> LocalAdoptionPolicyDescriptorV1;

    fn evaluate_local_adoption(
        &self,
        subject: &LocalAdoptionPolicySubjectV1<R>,
    ) -> Result<LocalAdoptionPolicyReceiptV1<R>, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalAdoptionPolicyError<E> {
    Evaluator(E),
    EvaluatorDescriptorMismatchBefore,
    EvaluatorDescriptorMismatchAfter,
    ActivePolicyMismatch,
    UnsupportedReceiptSchema,
    ReceiptEvaluatorMismatch,
    SubjectMismatch,
    ReceiptOutlivesEvaluator,
    ReceiptOutlivesReadiness,
    NonApprovalCarriesFollowOnAuthority,
}

struct BoundLocalPolicyEvidenceV1<P, EV, RV, AQ, LQ, const L: usize, const R: usize> {
    readiness: LocalAdoptionReadyEvidenceV1<EV, RV, AQ, LQ, L, R>,
    expected: ExpectedLocalAdoptionPolicyProfileV1,
    receipt: LocalAdoptionPolicyReceiptV1<R>,
    _policy_type: PhantomData<fn() -> P>,
}

pub struct ApprovedLocalAdoptionV1<P, EV, RV, AQ, LQ, const L: usize, const R: usize> {
    bound: BoundLocalPolicyEvidenceV1<P, EV, RV, AQ, LQ, L, R>,
}

pub struct RejectedLocalAdoptionV1<P, EV, RV, AQ, LQ, const L: usize, const R: usize> {
    bound: BoundLocalPolicyEvidenceV1<P, EV, RV, AQ, LQ, L, R>,
}

pub struct DeferredLocalAdoptionV1<P, EV, RV, AQ, LQ, const L: usize, const R: usize> {
    bound: BoundLocalPolicyEvidenceV1<P, EV, RV, AQ, LQ, L, R>,
}

macro_rules! impl_policy_outcome {
    ($name:ident) => {
        impl<P, EV, RV, AQ, LQ, const L: usize, const R: usize>
            $name<P, EV, RV, AQ, LQ, L, R>
        {
            pub const fn readiness(
                &self,
            ) -> &LocalAdoptionReadyEvidenceV1<EV, RV, AQ, LQ, L, R> {
                &self.bound.readiness
            }

            pub const fn expected_profile(&self) -> ExpectedLocalAdoptionPolicyProfileV1 {
                self.bound.expected
            }

            pub const fn receipt(&self) -> &LocalAdoptionPolicyReceiptV1<R> {
                &self.bound.receipt
            }

            pub const fn valid_until(&self) -> u64 {
                self.bound.receipt.valid_until
            }

            pub const fn adoption_scope(&self) -> LocalAdoptionScopeCommitment {
                self.bound.receipt.adoption_scope
            }

            pub const fn maximum_follow_on_consequence(&self) -> ConsequenceClass {
                self.bound.receipt.maximum_follow_on_consequence
            }

            pub const fn contains_effect_authority(&self) -> bool {
                false
            }

            pub const fn contains_state_install_authority(&self) -> bool {
                false
            }
        }
    };
}

impl_policy_outcome!(ApprovedLocalAdoptionV1);
impl_policy_outcome!(RejectedLocalAdoptionV1);
impl_policy_outcome!(DeferredLocalAdoptionV1);

pub enum LocalAdoptionPolicyResultV1<P, EV, RV, AQ, LQ, const L: usize, const R: usize> {
    Approved(ApprovedLocalAdoptionV1<P, EV, RV, AQ, LQ, L, R>),
    Rejected(RejectedLocalAdoptionV1<P, EV, RV, AQ, LQ, L, R>),
    Deferred(DeferredLocalAdoptionV1<P, EV, RV, AQ, LQ, L, R>),
}

fn non_approval_is_attenuated(
    decision: LocalAdoptionDecisionV1,
    maximum_follow_on_consequence: ConsequenceClass,
) -> bool {
    decision == LocalAdoptionDecisionV1::Approve
        || maximum_follow_on_consequence == ConsequenceClass::C0Observe
}

/// Evaluate one exact readiness token under an independently expected local
/// policy evaluator generation.
pub fn evaluate_local_adoption_policy<P, EV, RV, AQ, LQ, const L: usize, const R: usize>(
    expected: ExpectedLocalAdoptionPolicyProfileV1,
    evaluator: &P,
    readiness: LocalAdoptionReadyEvidenceV1<EV, RV, AQ, LQ, L, R>,
) -> Result<
    LocalAdoptionPolicyResultV1<P, EV, RV, AQ, LQ, L, R>,
    LocalAdoptionPolicyError<P::Error>,
>
where
    P: LocalAdoptionPolicyEvaluatorV1<R>,
{
    let expected_descriptor = expected.descriptor();
    if evaluator.descriptor() != expected_descriptor {
        return Err(LocalAdoptionPolicyError::EvaluatorDescriptorMismatchBefore);
    }

    let subject = subject_from_readiness(&readiness);
    if expected_descriptor.policy_digest != subject.local_state.context().policy_digest {
        return Err(LocalAdoptionPolicyError::ActivePolicyMismatch);
    }

    let receipt = evaluator
        .evaluate_local_adoption(&subject)
        .map_err(LocalAdoptionPolicyError::Evaluator)?;

    if evaluator.descriptor() != expected_descriptor {
        return Err(LocalAdoptionPolicyError::EvaluatorDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(LocalAdoptionPolicyError::UnsupportedReceiptSchema);
    }
    if receipt.evaluator != expected_descriptor {
        return Err(LocalAdoptionPolicyError::ReceiptEvaluatorMismatch);
    }
    if receipt.subject != subject {
        return Err(LocalAdoptionPolicyError::SubjectMismatch);
    }
    if receipt.valid_until > expected_descriptor.valid_until {
        return Err(LocalAdoptionPolicyError::ReceiptOutlivesEvaluator);
    }
    if receipt.valid_until > readiness.valid_until() {
        return Err(LocalAdoptionPolicyError::ReceiptOutlivesReadiness);
    }
    if !non_approval_is_attenuated(receipt.decision, receipt.maximum_follow_on_consequence) {
        return Err(LocalAdoptionPolicyError::NonApprovalCarriesFollowOnAuthority);
    }

    let bound = BoundLocalPolicyEvidenceV1 {
        readiness,
        expected,
        receipt,
        _policy_type: PhantomData,
    };

    Ok(match bound.receipt.decision {
        LocalAdoptionDecisionV1::Approve => {
            LocalAdoptionPolicyResultV1::Approved(ApprovedLocalAdoptionV1 { bound })
        }
        LocalAdoptionDecisionV1::Reject => {
            LocalAdoptionPolicyResultV1::Rejected(RejectedLocalAdoptionV1 { bound })
        }
        LocalAdoptionDecisionV1::Defer => {
            LocalAdoptionPolicyResultV1::Deferred(DeferredLocalAdoptionV1 { bound })
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn policy_outcomes_are_closed_world_and_distinct() {
        assert_ne!(LocalAdoptionDecisionV1::Approve, LocalAdoptionDecisionV1::Reject);
        assert_ne!(LocalAdoptionDecisionV1::Approve, LocalAdoptionDecisionV1::Defer);
        assert_ne!(LocalAdoptionDecisionV1::Reject, LocalAdoptionDecisionV1::Defer);
    }

    #[test]
    fn reject_and_defer_cannot_carry_follow_on_power() {
        assert!(non_approval_is_attenuated(
            LocalAdoptionDecisionV1::Reject,
            ConsequenceClass::C0Observe,
        ));
        assert!(non_approval_is_attenuated(
            LocalAdoptionDecisionV1::Defer,
            ConsequenceClass::C0Observe,
        ));
        assert!(!non_approval_is_attenuated(
            LocalAdoptionDecisionV1::Reject,
            ConsequenceClass::C3ScopedRestriction,
        ));
        assert!(!non_approval_is_attenuated(
            LocalAdoptionDecisionV1::Defer,
            ConsequenceClass::C3ScopedRestriction,
        ));
    }
}

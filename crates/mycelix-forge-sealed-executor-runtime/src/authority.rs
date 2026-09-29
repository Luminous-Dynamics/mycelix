// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-010: deliberate final authority join for protected-ref execution.
//!
//! This module is intentionally colocated with the sealed executor runtime so
//! it can construct the opaque MergeExecutionAuthorizationV1. It joins the
//! collaboration/authority and source/execution chains only at this final gate.
//!
//! The prepared-execution permit is deliberately excluded: it is issued only
//! after durable preparation has committed and therefore belongs to the
//! execution coordinator lifecycle, not the static authority theorem.

use forge_git_ref_transaction::GitRefTransactionPlanV1;
use forge_gittuf_marker_policy::GittufMarkerNamespacePolicyEvidenceV1;
use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_m0_offline_evidence_v6::QualifiedM0OfflineEvidenceV6;
use mycelix_forge_protected_merge_source::SourceQualifiedProtectedMergeRequestV1;
use mycelix_forge_protected_ref_consumption::AtomicProtectedRefConsumptionIntentV1;
use mycelix_forge_protected_ref_executor_confinement::ExecutorConstrainedGitRefTransactionPlanV1;
use mycelix_forge_review_basis_currentness_trust_context::MergeProtectedTrustedReviewBasisCurrentnessV1;
use mycelix_forge_sealed_executor_interface::{
    SealedExecutorInterfaceBindingV1, SealedExecutorInvocationV1,
};
use mycelix_forge_proposal::ChangeProposalId;
use serde::Serialize;
use thiserror::Error;

use crate::{
    MergeExecutionAuthorizationV1, SealedExecutorRuntimeConfigV1, SealedExecutorRuntimeError,
};

const AUTHORITY_BASIS_DOMAIN_V1: &[u8] =
    b"mycelix-forge/merge-execution-authority-basis/v1\0";
const AUTHORIZATION_EVIDENCE_DOMAIN_V1: &[u8] =
    b"mycelix-forge/merge-execution-authorization/v1\0";

/// Serializable evidence describing the exact authority join that produced one
/// opaque merge-execution authorization.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MergeExecutionAuthorityEvidenceV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    review_currentness: Digest,
    source_qualification: Digest,
    atomic_intent: Digest,
    repository_request: Digest,
    repository_policy_state: Digest,
    m0_evidence: Digest,
    m0_execution_subject: Digest,
    marker_policy_evidence: Digest,
    executor_confinement_evidence: Digest,
    sealed_interface_evidence: Digest,
    invocation_commitment: Digest,
    runtime_config_commitment: Digest,
    authority_observed_at_unix_ms: u64,
    authority_evidence: Digest,
    evidence_commitment: Digest,
}

impl MergeExecutionAuthorityEvidenceV1 {
    /// Exact project.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected-review currentness trust evidence.
    pub fn review_currentness(&self) -> &Digest {
        &self.review_currentness
    }

    /// Exact source-qualification evidence.
    pub fn source_qualification(&self) -> &Digest {
        &self.source_qualification
    }

    /// Exact atomic transaction intent.
    pub fn atomic_intent(&self) -> &Digest {
        &self.atomic_intent
    }

    /// Exact repository verification request.
    pub fn repository_request(&self) -> &Digest {
        &self.repository_request
    }

    /// Exact repository policy state.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact M0 offline-evidence commitment.
    pub fn m0_evidence(&self) -> &Digest {
        &self.m0_evidence
    }

    /// Exact M0 execution-subject commitment.
    pub fn m0_execution_subject(&self) -> &Digest {
        &self.m0_execution_subject
    }

    /// Exact gittuf marker-policy evidence.
    pub fn marker_policy_evidence(&self) -> &Digest {
        &self.marker_policy_evidence
    }

    /// Exact closed-world executor-confinement evidence.
    pub fn executor_confinement_evidence(&self) -> &Digest {
        &self.executor_confinement_evidence
    }

    /// Exact sealed-executor interface evidence.
    pub fn sealed_interface_evidence(&self) -> &Digest {
        &self.sealed_interface_evidence
    }

    /// Exact canonical invocation commitment.
    pub fn invocation_commitment(&self) -> &Digest {
        &self.invocation_commitment
    }

    /// Exact provider-fixed runtime configuration commitment.
    pub fn runtime_config_commitment(&self) -> &Digest {
        &self.runtime_config_commitment
    }

    /// Authority observation time used for the exact currentness freshness binding.
    pub const fn authority_observed_at_unix_ms(&self) -> u64 {
        self.authority_observed_at_unix_ms
    }

    /// Digest of all joined authority evidence before the outer authorization wrapper.
    pub fn authority_evidence(&self) -> &Digest {
        &self.authority_evidence
    }

    /// Outer authorization evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Construct the opaque final merge-execution authorization only after the
/// complete collaboration/source/execution/M0 theorem succeeds.
///
/// The function performs explicit cross-chain joins rather than assuming that
/// positive child types are automatically about the same proposal or plan.
#[allow(clippy::too_many_arguments)]
pub fn qualify_merge_execution_authorization_v1(
    review: &MergeProtectedTrustedReviewBasisCurrentnessV1,
    source: &SourceQualifiedProtectedMergeRequestV1,
    intent: &AtomicProtectedRefConsumptionIntentV1,
    m0: &QualifiedM0OfflineEvidenceV6,
    marker_policy: &GittufMarkerNamespacePolicyEvidenceV1,
    constrained_plan: &ExecutorConstrainedGitRefTransactionPlanV1,
    plan: &GitRefTransactionPlanV1,
    binding: &SealedExecutorInterfaceBindingV1,
    invocation: &SealedExecutorInvocationV1,
    runtime_config: &SealedExecutorRuntimeConfigV1,
    authority_observed_at_unix_ms: u64,
) -> Result<
    (MergeExecutionAuthorizationV1, MergeExecutionAuthorityEvidenceV1),
    MergeExecutionAuthorityError,
> {
    if source.project() != review.project() || intent.project() != review.project() {
        return Err(MergeExecutionAuthorityError::ProjectMismatch);
    }
    if source.proposal() != review.proposal() || intent.proposal() != review.proposal() {
        return Err(MergeExecutionAuthorityError::ProposalMismatch);
    }

    if source.evidence_commitment() != intent.source_qualification() {
        return Err(MergeExecutionAuthorityError::SourceQualificationMismatch);
    }

    let intent_algorithm = plan.atomic_intent().algorithm();
    let expected_atomic_intent = intent
        .digest(intent_algorithm)
        .map_err(MergeExecutionAuthorityError::AtomicIntent)?;
    if plan.atomic_intent() != &expected_atomic_intent {
        return Err(MergeExecutionAuthorityError::AtomicIntentMismatch);
    }

    if plan.target_ref() != intent.target_ref()
        || plan.expected_base() != intent.expected_base()
        || plan.proposed_revision() != intent.proposed_revision()
        || plan.consumption_marker_ref() != intent.consumption_marker_ref()
        || plan.consumption_marker_value() != intent.consumption_marker_value()
    {
        return Err(MergeExecutionAuthorityError::TransactionPlanMismatch);
    }

    let m0_repository = m0.repository().structural();
    if m0_repository.request_digest() != source.repository_request() {
        return Err(MergeExecutionAuthorityError::M0RepositoryRequestMismatch);
    }
    if m0_repository.observed_tip() != intent.proposed_revision() {
        return Err(MergeExecutionAuthorityError::M0ObservedTipMismatch);
    }
    if m0_repository.repository_policy_state() != review.repository_policy_state() {
        return Err(MergeExecutionAuthorityError::M0RepositoryPolicyMismatch);
    }
    if source.pending_offline_evidence() != m0.evidence_commitment() {
        return Err(MergeExecutionAuthorityError::M0EvidenceMismatch);
    }
    if source.pending_execution_subject() != m0.execution_subject() {
        return Err(MergeExecutionAuthorityError::M0ExecutionSubjectMismatch);
    }

    if marker_policy.project() != review.project()
        || marker_policy.marker_ref() != plan.consumption_marker_ref()
    {
        return Err(MergeExecutionAuthorityError::MarkerPolicyMismatch);
    }
    if constrained_plan.project() != review.project()
        || constrained_plan.plan_evidence() != plan.evidence_commitment()
        || constrained_plan.marker_policy_evidence() != marker_policy.evidence_commitment()
    {
        return Err(MergeExecutionAuthorityError::ExecutorConfinementMismatch);
    }

    if binding.project() != review.project()
        || binding.plan_evidence() != plan.evidence_commitment()
        || binding.executor_identity() != runtime_config.executor_identity()
        || binding.repository_identity() != runtime_config.repository_identity()
    {
        return Err(MergeExecutionAuthorityError::SealedBindingMismatch);
    }

    if !constrained_plan.bindings().iter().any(|candidate| {
        candidate.principal_id() == binding.principal_id()
            && candidate.executor_identity() == binding.executor_identity()
            && candidate.allowed_plan() == binding.plan_evidence()
    }) {
        return Err(MergeExecutionAuthorityError::ExecutorBindingNotInClosedWorld);
    }

    let expected_invocation = SealedExecutorInvocationV1::from_binding(binding, plan)
        .map_err(MergeExecutionAuthorityError::Invocation)?;
    let invocation_algorithm = invocation.plan_evidence().algorithm();
    let expected_invocation_digest = expected_invocation
        .digest(invocation_algorithm)
        .map_err(MergeExecutionAuthorityError::Invocation)?;
    let actual_invocation_digest = invocation
        .digest(invocation_algorithm)
        .map_err(MergeExecutionAuthorityError::Invocation)?;
    if actual_invocation_digest != expected_invocation_digest {
        return Err(MergeExecutionAuthorityError::InvocationMismatch);
    }
    if invocation.executor_identity() != runtime_config.executor_identity()
        || invocation.repository_identity() != runtime_config.repository_identity()
        || invocation.plan_evidence() != plan.evidence_commitment()
    {
        return Err(MergeExecutionAuthorityError::InvocationMismatch);
    }

    if authority_observed_at_unix_ms != review.currentness_observed_at_unix_ms() {
        return Err(MergeExecutionAuthorityError::CurrentnessFreshnessMismatch);
    }

    let authority_algorithm = review.evidence_commitment().algorithm();
    let invocation_commitment = invocation
        .digest(authority_algorithm)
        .map_err(MergeExecutionAuthorityError::Invocation)?;
    let runtime_config_commitment = runtime_config
        .commitment(authority_algorithm)
        .map_err(MergeExecutionAuthorityError::RuntimeConfig)?;

    let mut basis = Vec::new();
    basis.extend_from_slice(AUTHORITY_BASIS_DOMAIN_V1);
    basis.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project(&mut basis, review.project())?;
    push_digest(&mut basis, review.proposal().commitment())?;
    push_digest(&mut basis, review.evidence_commitment())?;
    push_digest(&mut basis, source.evidence_commitment())?;
    push_digest(&mut basis, &expected_atomic_intent)?;
    push_digest(&mut basis, source.repository_request())?;
    push_digest(&mut basis, review.repository_policy_state())?;
    push_digest(&mut basis, m0.evidence_commitment())?;
    push_digest(&mut basis, m0.execution_subject())?;
    push_digest(&mut basis, marker_policy.evidence_commitment())?;
    push_digest(&mut basis, constrained_plan.evidence_commitment())?;
    push_digest(&mut basis, binding.evidence_commitment())?;
    push_digest(&mut basis, &invocation_commitment)?;
    push_digest(&mut basis, &runtime_config_commitment)?;
    basis.extend_from_slice(&authority_observed_at_unix_ms.to_be_bytes());
    let authority_evidence = Digest::of_bytes(authority_algorithm, &basis);

    let mut outer = Vec::new();
    outer.extend_from_slice(AUTHORIZATION_EVIDENCE_DOMAIN_V1);
    outer.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut outer, &authority_evidence)?;
    push_digest(&mut outer, &invocation_commitment)?;
    push_digest(&mut outer, &runtime_config_commitment)?;
    outer.extend_from_slice(&authority_observed_at_unix_ms.to_be_bytes());
    let evidence_commitment = Digest::of_bytes(authority_algorithm, &outer);

    let authorization = MergeExecutionAuthorizationV1 {
        invocation_commitment,
        runtime_config_commitment,
        authority_evidence: authority_evidence.clone(),
        evidence_commitment: evidence_commitment.clone(),
    };

    let evidence = MergeExecutionAuthorityEvidenceV1 {
        project: review.project().clone(),
        proposal: review.proposal().clone(),
        review_currentness: review.evidence_commitment().clone(),
        source_qualification: source.evidence_commitment().clone(),
        atomic_intent: expected_atomic_intent,
        repository_request: source.repository_request().clone(),
        repository_policy_state: review.repository_policy_state().clone(),
        m0_evidence: m0.evidence_commitment().clone(),
        m0_execution_subject: m0.execution_subject().clone(),
        marker_policy_evidence: marker_policy.evidence_commitment().clone(),
        executor_confinement_evidence: constrained_plan.evidence_commitment().clone(),
        sealed_interface_evidence: binding.evidence_commitment().clone(),
        invocation_commitment,
        runtime_config_commitment,
        authority_observed_at_unix_ms,
        authority_evidence,
        evidence_commitment,
    };

    Ok((authorization, evidence))
}

fn push_project(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), MergeExecutionAuthorityError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), MergeExecutionAuthorityError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| MergeExecutionAuthorityError::CanonicalFieldTooLarge("digest algorithm"))?;
    let bytes = digest.as_bytes();
    let digest_len = u32::try_from(bytes.len())
        .map_err(|_| MergeExecutionAuthorityError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Final authority-join failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum MergeExecutionAuthorityError {
    /// Inputs name different projects.
    #[error("merge execution authority project mismatch")]
    ProjectMismatch,
    /// Inputs name different proposals.
    #[error("merge execution authority proposal mismatch")]
    ProposalMismatch,
    /// Source qualification does not match the atomic intent.
    #[error("merge execution authority source qualification mismatch")]
    SourceQualificationMismatch,
    /// Atomic intent digest could not be computed.
    #[error("merge execution authority atomic intent error: {0}")]
    AtomicIntent(
        mycelix_forge_protected_ref_consumption::ProtectedRefConsumptionError,
    ),
    /// Atomic intent does not match the exact transaction plan.
    #[error("merge execution authority atomic intent mismatch")]
    AtomicIntentMismatch,
    /// Transaction plan differs from the exact atomic intent.
    #[error("merge execution authority transaction plan mismatch")]
    TransactionPlanMismatch,
    /// M0 repository request differs from source qualification.
    #[error("merge execution authority M0 repository request mismatch")]
    M0RepositoryRequestMismatch,
    /// M0 observed tip differs from the requested new revision.
    #[error("merge execution authority M0 observed tip mismatch")]
    M0ObservedTipMismatch,
    /// M0 repository policy differs from the protected review policy state.
    #[error("merge execution authority M0 repository policy mismatch")]
    M0RepositoryPolicyMismatch,
    /// M0 evidence does not match the pending source reference.
    #[error("merge execution authority M0 evidence mismatch")]
    M0EvidenceMismatch,
    /// M0 execution subject does not match the pending source reference.
    #[error("merge execution authority M0 execution subject mismatch")]
    M0ExecutionSubjectMismatch,
    /// Marker policy does not bind the exact transaction marker.
    #[error("merge execution authority marker policy mismatch")]
    MarkerPolicyMismatch,
    /// Executor-confinement evidence does not bind the exact plan/policy.
    #[error("merge execution authority executor confinement mismatch")]
    ExecutorConfinementMismatch,
    /// Sealed executor binding does not bind the exact project/plan/runtime identities.
    #[error("merge execution authority sealed binding mismatch")]
    SealedBindingMismatch,
    /// The binding principal is outside the closed-world executor set.
    #[error("merge execution authority binding principal is outside closed-world set")]
    ExecutorBindingNotInClosedWorld,
    /// Sealed invocation cannot be reconstructed or digested.
    #[error("merge execution authority invocation error: {0}")]
    Invocation(mycelix_forge_sealed_executor_interface::SealedExecutorInterfaceError),
    /// Supplied invocation differs from the exact binding-derived invocation.
    #[error("merge execution authority invocation mismatch")]
    InvocationMismatch,
    /// Provider currentness is not fresh at the authority observation point.
    #[error("merge execution authority currentness freshness mismatch")]
    CurrentnessFreshnessMismatch,
    /// Runtime configuration commitment failed.
    #[error("merge execution authority runtime configuration error: {0}")]
    RuntimeConfig(SealedExecutorRuntimeError),
    /// Canonical authority evidence field was too large.
    #[error("merge execution authority canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn authority_api_returns_opaque_authorization_and_exportable_evidence() {
        fn assert_signature(
            f: fn(
                &MergeProtectedTrustedReviewBasisCurrentnessV1,
                &SourceQualifiedProtectedMergeRequestV1,
                &AtomicProtectedRefConsumptionIntentV1,
                &QualifiedM0OfflineEvidenceV6,
                &GittufMarkerNamespacePolicyEvidenceV1,
                &ExecutorConstrainedGitRefTransactionPlanV1,
                &GitRefTransactionPlanV1,
                &SealedExecutorInterfaceBindingV1,
                &SealedExecutorInvocationV1,
                &SealedExecutorRuntimeConfigV1,
                u64,
            ) -> Result<
                (MergeExecutionAuthorizationV1, MergeExecutionAuthorityEvidenceV1),
                MergeExecutionAuthorityError,
            >,
        ) {}
        assert_signature(qualify_merge_execution_authorization_v1);
    }
}

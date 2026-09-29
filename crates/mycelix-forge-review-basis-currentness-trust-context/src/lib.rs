// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-007P: make the trust decision for provider-scoped review-basis
//! currentness explicit in the exact MergeProtected finalization context.
//!
//! The theorem is:
//!
//! MergeProtectedReviewBasisQuorumV1
//! + exact finalization_context
//! + ReviewBasisCurrentnessTrustContextV1
//! + ProviderVerifiedReviewBasisCurrentnessV1
//! + exact provider namespace
//! + exact verifier identity
//! + exact coverage commitment
//!     -> MergeProtectedTrustedReviewBasisCurrentnessV1
//!
//! No unrelated project-policy authentication trust root is reused.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_merge_protected_review_basis::MergeProtectedReviewBasisQuorumV1;
use mycelix_forge_review_basis_currentness::{
    ProviderVerifiedReviewBasisCurrentnessV1, ReviewBasisCurrentnessVerifierIdentityV1,
};
use mycelix_forge_proposal::ChangeProposalId;
use serde::{Deserialize, Serialize};
use thiserror::Error;

const TRUST_CONTEXT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-basis-currentness-trust-context/v1\0";
const TRUSTED_CURRENTNESS_DOMAIN_V1: &[u8] =
    b"mycelix-forge/merge-protected-trusted-review-basis-currentness/v1\0";
const MAX_STRING_LEN: usize = 512;

/// Exact provider/verifier/coverage context explicitly selected by protected
/// merge authority for review-basis currentness.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewBasisCurrentnessTrustContextV1 {
    version: ProtocolVersion,
    provider_namespace: String,
    verifier: ReviewBasisCurrentnessVerifierIdentityV1,
    coverage_commitment: Digest,
}

impl ReviewBasisCurrentnessTrustContextV1 {
    /// Construct a bounded currentness trust context.
    pub fn new(
        provider_namespace: impl Into<String>,
        verifier: ReviewBasisCurrentnessVerifierIdentityV1,
        coverage_commitment: Digest,
    ) -> Result<Self, ReviewBasisCurrentnessTrustContextError> {
        let provider_namespace = provider_namespace.into();
        validate_string(&provider_namespace, "provider namespace")?;
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            provider_namespace,
            verifier,
            coverage_commitment,
        })
    }

    /// Exact provider namespace selected by protected authority.
    pub fn provider_namespace(&self) -> &str {
        &self.provider_namespace
    }

    /// Exact currentness verifier identity selected by protected authority.
    pub fn verifier(&self) -> &ReviewBasisCurrentnessVerifierIdentityV1 {
        &self.verifier
    }

    /// Exact provider-declared currentness coverage commitment selected by protected authority.
    pub fn coverage_commitment(&self) -> &Digest {
        &self.coverage_commitment
    }

    /// Canonical commitment to this exact trust context.
    pub fn commitment(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ReviewBasisCurrentnessTrustContextError> {
        let mut out = Vec::new();
        out.extend_from_slice(TRUST_CONTEXT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_string(&mut out, &self.provider_namespace)?;
        push_string(&mut out, self.verifier.name())?;
        push_string(&mut out, self.verifier.version())?;
        push_digest(&mut out, &self.coverage_commitment)?;
        Ok(Digest::of_bytes(algorithm, &out))
    }
}

/// Positive evidence that protected merge authority explicitly trusted one exact
/// provider/verifier/coverage context for one exact review-basis quorum.
///
/// This is not a global currentness oracle and is intentionally not deserializable
/// into positive authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MergeProtectedTrustedReviewBasisCurrentnessV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    review_basis_quorum: Digest,
    finalization_context: Digest,
    currentness_evidence: Digest,
    provider_namespace: String,
    verifier: ReviewBasisCurrentnessVerifierIdentityV1,
    coverage_commitment: Digest,
    currentness_observed_at_unix_ms: u64,
    evidence_commitment: Digest,
}

impl MergeProtectedTrustedReviewBasisCurrentnessV1 {
    /// Exact project.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected review-basis quorum evidence.
    pub fn review_basis_quorum(&self) -> &Digest {
        &self.review_basis_quorum
    }

    /// Exact finalization context selected by protected merge authority.
    pub fn finalization_context(&self) -> &Digest {
        &self.finalization_context
    }

    /// Exact provider-scoped currentness evidence.
    pub fn currentness_evidence(&self) -> &Digest {
        &self.currentness_evidence
    }

    /// Exact provider namespace selected by protected authority.
    pub fn provider_namespace(&self) -> &str {
        &self.provider_namespace
    }

    /// Exact currentness verifier identity selected by protected authority.
    pub fn verifier(&self) -> &ReviewBasisCurrentnessVerifierIdentityV1 {
        &self.verifier
    }

    /// Exact provider coverage commitment selected by protected authority.
    pub fn coverage_commitment(&self) -> &Digest {
        &self.coverage_commitment
    }

    /// Provider currentness observation time.
    pub const fn currentness_observed_at_unix_ms(&self) -> u64 {
        self.currentness_observed_at_unix_ms
    }

    /// Aggregate evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind provider-scoped currentness to the exact trust context selected by the
/// MergeProtected review-basis quorum.
pub fn bind_merge_protected_trusted_review_basis_currentness_v1(
    quorum: &MergeProtectedReviewBasisQuorumV1,
    context: &ReviewBasisCurrentnessTrustContextV1,
    currentness: &ProviderVerifiedReviewBasisCurrentnessV1,
) -> Result<
    MergeProtectedTrustedReviewBasisCurrentnessV1,
    ReviewBasisCurrentnessTrustContextError,
> {
    let context_commitment = context.commitment(quorum.finalization_context().algorithm())?;
    if context_commitment != *quorum.finalization_context() {
        return Err(ReviewBasisCurrentnessTrustContextError::FinalizationContextMismatch);
    }
    if currentness.review_basis_quorum() != quorum.evidence_commitment() {
        return Err(ReviewBasisCurrentnessTrustContextError::ReviewBasisQuorumMismatch);
    }
    if currentness.provider_namespace() != context.provider_namespace() {
        return Err(ReviewBasisCurrentnessTrustContextError::ProviderNamespaceMismatch);
    }
    if currentness.verifier() != context.verifier() {
        return Err(ReviewBasisCurrentnessTrustContextError::VerifierIdentityMismatch);
    }
    if currentness.coverage_commitment() != context.coverage_commitment() {
        return Err(ReviewBasisCurrentnessTrustContextError::CoverageMismatch);
    }
    if currentness.observed_at_unix_ms() < quorum.quorum_observed_at_unix_ms() {
        return Err(ReviewBasisCurrentnessTrustContextError::ObservationBeforeQuorum);
    }

    let algorithm = currentness.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(TRUSTED_CURRENTNESS_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, quorum.project())?;
    push_digest(&mut out, quorum.proposal().commitment())?;
    push_digest(&mut out, quorum.authority_epoch())?;
    push_digest(&mut out, quorum.project_policy())?;
    push_digest(&mut out, quorum.repository_policy_state())?;
    push_digest(&mut out, quorum.evidence_commitment())?;
    push_digest(&mut out, &context_commitment)?;
    push_digest(&mut out, currentness.evidence_commitment())?;
    push_string(&mut out, context.provider_namespace())?;
    push_string(&mut out, context.verifier().name())?;
    push_string(&mut out, context.verifier().version())?;
    push_digest(&mut out, context.coverage_commitment())?;
    out.extend_from_slice(&currentness.observed_at_unix_ms().to_be_bytes());

    Ok(MergeProtectedTrustedReviewBasisCurrentnessV1 {
        project: quorum.project().clone(),
        proposal: quorum.proposal().clone(),
        review_basis_quorum: quorum.evidence_commitment().clone(),
        finalization_context: context_commitment,
        currentness_evidence: currentness.evidence_commitment().clone(),
        provider_namespace: context.provider_namespace().to_owned(),
        verifier: context.verifier().clone(),
        coverage_commitment: context.coverage_commitment().clone(),
        currentness_observed_at_unix_ms: currentness.observed_at_unix_ms(),
        evidence_commitment: Digest::of_bytes(algorithm, &out),
    })
}

fn validate_string(
    value: &str,
    field: &'static str,
) -> Result<(), ReviewBasisCurrentnessTrustContextError> {
    if value.is_empty() || value.len() > MAX_STRING_LEN {
        return Err(ReviewBasisCurrentnessTrustContextError::InvalidString(field));
    }
    Ok(())
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewBasisCurrentnessTrustContextError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ReviewBasisCurrentnessTrustContextError> {
    push_string(out, digest.algorithm().id())?;
    let len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| ReviewBasisCurrentnessTrustContextError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
) -> Result<(), ReviewBasisCurrentnessTrustContextError> {
    let len = u32::try_from(value.len())
        .map_err(|_| ReviewBasisCurrentnessTrustContextError::CanonicalFieldTooLarge("string"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

/// Trust-context qualification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewBasisCurrentnessTrustContextError {
    /// The typed context commitment differs from MergeProtected finalization_context.
    #[error("review-basis currentness trust context differs from finalization context")]
    FinalizationContextMismatch,
    /// The currentness evidence names another review-basis quorum.
    #[error("review-basis currentness evidence names another quorum")]
    ReviewBasisQuorumMismatch,
    /// The currentness provider namespace differs from the selected context.
    #[error("review-basis currentness provider namespace differs from trust context")]
    ProviderNamespaceMismatch,
    /// The currentness verifier differs from the selected context.
    #[error("review-basis currentness verifier differs from trust context")]
    VerifierIdentityMismatch,
    /// The currentness coverage differs from the selected context.
    #[error("review-basis currentness coverage differs from trust context")]
    CoverageMismatch,
    /// The currentness observation predates the protected quorum.
    #[error("review-basis currentness observation predates protected quorum")]
    ObservationBeforeQuorum,
    /// A bounded string is invalid.
    #[error("review-basis currentness trust context invalid {0}")]
    InvalidString(&'static str),
    /// A canonical field exceeded its encoding bound.
    #[error("review-basis currentness trust context canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    #[test]
    fn binding_api_requires_exact_quorum_context_and_currentness_inputs() {
        let function: fn(
            &MergeProtectedReviewBasisQuorumV1,
            &ReviewBasisCurrentnessTrustContextV1,
            &ProviderVerifiedReviewBasisCurrentnessV1,
        ) -> Result<
            MergeProtectedTrustedReviewBasisCurrentnessV1,
            ReviewBasisCurrentnessTrustContextError,
        > = bind_merge_protected_trusted_review_basis_currentness_v1;
        let _ = function;
    }

    #[test]
    fn trust_context_commitment_is_deterministic() {
        let context = ReviewBasisCurrentnessTrustContextV1::new(
            "provider-a",
            ReviewBasisCurrentnessVerifierIdentityV1::new("verifier-a", "v1").unwrap(),
            digest(1),
        )
        .unwrap();
        assert_eq!(
            context.commitment(DigestAlgorithm::Sha256).unwrap(),
            context.commitment(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn changing_coverage_changes_context_commitment() {
        let a = ReviewBasisCurrentnessTrustContextV1::new(
            "provider-a",
            ReviewBasisCurrentnessVerifierIdentityV1::new("verifier-a", "v1").unwrap(),
            digest(1),
        )
        .unwrap();
        let b = ReviewBasisCurrentnessTrustContextV1::new(
            "provider-a",
            ReviewBasisCurrentnessVerifierIdentityV1::new("verifier-a", "v1").unwrap(),
            digest(2),
        )
        .unwrap();
        assert_ne!(
            a.commitment(DigestAlgorithm::Sha256).unwrap(),
            b.commitment(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn trust_context_is_wire_serializable_but_positive_result_is_not_deserializable() {
        let context = ReviewBasisCurrentnessTrustContextV1::new(
            "provider-a",
            ReviewBasisCurrentnessVerifierIdentityV1::new("verifier-a", "v1").unwrap(),
            digest(1),
        )
        .unwrap();
        let bytes = serde_json::to_vec(&context).unwrap();
        let decoded: ReviewBasisCurrentnessTrustContextV1 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(context, decoded);
    }
}

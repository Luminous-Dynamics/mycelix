// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-007P: bind provider-scoped review-basis currentness to the exact
//! project policy and provider/verifier trust entry committed by that policy.
//!
//! The trust theorem is:
//!
//! ```text
//! MergeProtectedReviewBasisQuorumV1
//! + ProviderVerifiedReviewBasisCurrentnessV1
//! + exact ProjectPolicyStateV1
//! + exact AuthenticationProviderTrustPolicyV1
//! + project-policy binding
//! + exact provider-namespace commitment
//! + exact verifier-identity commitment
//! + explicit project-policy trust entry
//!     -> ProjectPolicyTrustedReviewBasisCurrentnessV1
//! ```
//!
//! This is a trust join, not a global current-state oracle.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_merge_protected_review_basis::MergeProtectedReviewBasisQuorumV1;
use mycelix_forge_project_policy::{
    AuthenticationProviderTrustPolicyV1, ProjectPolicyError, ProjectPolicyStateV1,
};
use mycelix_forge_proposal::ChangeProposalId;
use mycelix_forge_review_basis_currentness::{
    ProviderVerifiedReviewBasisCurrentnessV1, ReviewBasisCurrentnessVerifierIdentityV1,
};
use serde::Serialize;
use thiserror::Error;

const PROVIDER_NAMESPACE_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-basis-currentness-provider-namespace/v1\0";
const VERIFIER_IDENTITY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-basis-currentness-verifier-identity/v1\0";
const TRUSTED_CURRENTNESS_DOMAIN_V1: &[u8] =
    b"mycelix-forge/project-policy-trusted-review-basis-currentness/v1\0";
const MAX_STRING_LEN: usize = 512;

/// Canonical commitment to one exact currentness provider namespace.
pub fn provider_namespace_commitment_v1(
    namespace: &str,
    algorithm: DigestAlgorithm,
) -> Result<Digest, ReviewBasisCurrentnessPolicyTrustError> {
    validate_string(namespace, "provider namespace")?;
    let mut out = Vec::new();
    out.extend_from_slice(PROVIDER_NAMESPACE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_string(&mut out, namespace)?;
    Ok(Digest::of_bytes(algorithm, &out))
}

/// Canonical commitment to one exact currentness verifier identity.
pub fn verifier_identity_commitment_v1(
    identity: &ReviewBasisCurrentnessVerifierIdentityV1,
    algorithm: DigestAlgorithm,
) -> Result<Digest, ReviewBasisCurrentnessPolicyTrustError> {
    validate_string(identity.name(), "verifier name")?;
    validate_string(identity.version(), "verifier version")?;
    let mut out = Vec::new();
    out.extend_from_slice(VERIFIER_IDENTITY_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_string(&mut out, identity.name())?;
    push_string(&mut out, identity.version())?;
    Ok(Digest::of_bytes(algorithm, &out))
}

/// Positive result proving that the exact provider currentness verifier is
/// trusted by the exact project policy committed by the protected review basis.
///
/// This type is serializable for evidence export but intentionally not
/// deserializable into positive authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectPolicyTrustedReviewBasisCurrentnessV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    review_basis_quorum: Digest,
    currentness_evidence: Digest,
    provider_namespace: Digest,
    verifier_identity: Digest,
    provider_trust_policy: Digest,
    currentness_observed_at_unix_ms: u64,
    coverage_commitment: Digest,
    evidence_commitment: Digest,
}

impl ProjectPolicyTrustedReviewBasisCurrentnessV1 {
    /// Exact project.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal selected by protected merge authority.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch bound by the review-basis quorum.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state commitment.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state commitment.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact protected review-basis quorum evidence commitment.
    pub fn review_basis_quorum(&self) -> &Digest {
        &self.review_basis_quorum
    }

    /// Exact project-policy-trusted currentness evidence commitment.
    pub fn currentness_evidence(&self) -> &Digest {
        &self.currentness_evidence
    }

    /// Exact provider namespace commitment used for the project-policy trust lookup.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Exact currentness verifier identity commitment used for the project-policy trust lookup.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Exact provider-trust policy commitment bound by project policy.
    pub fn provider_trust_policy(&self) -> &Digest {
        &self.provider_trust_policy
    }

    /// Provider observation time carried by the currentness evidence.
    pub const fn currentness_observed_at_unix_ms(&self) -> u64 {
        self.currentness_observed_at_unix_ms
    }

    /// Exact provider-declared currentness coverage commitment.
    pub fn coverage_commitment(&self) -> &Digest {
        &self.coverage_commitment
    }

    /// Aggregate evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one provider-verified currentness result to the exact project policy
/// and provider/verifier trust entry committed by that policy.
pub fn bind_project_policy_trusted_review_basis_currentness_v1(
    quorum: &MergeProtectedReviewBasisQuorumV1,
    project_policy: &ProjectPolicyStateV1,
    provider_trust: &AuthenticationProviderTrustPolicyV1,
    currentness: &ProviderVerifiedReviewBasisCurrentnessV1,
) -> Result<
    ProjectPolicyTrustedReviewBasisCurrentnessV1,
    ReviewBasisCurrentnessPolicyTrustError,
> {
    if project_policy.project() != quorum.project()
        || provider_trust.project() != quorum.project()
    {
        return Err(ReviewBasisCurrentnessPolicyTrustError::ProjectMismatch);
    }

    let expected_project_policy = project_policy
        .digest(quorum.project_policy().algorithm())
        .map_err(ReviewBasisCurrentnessPolicyTrustError::ProjectPolicy)?;
    if quorum.project_policy() != &expected_project_policy {
        return Err(ReviewBasisCurrentnessPolicyTrustError::ProjectPolicyMismatch);
    }

    if !project_policy
        .binds_provider_trust(provider_trust)
        .map_err(ReviewBasisCurrentnessPolicyTrustError::ProjectPolicy)?
    {
        return Err(ReviewBasisCurrentnessPolicyTrustError::ProviderTrustPolicyMismatch);
    }

    if currentness.review_basis_quorum() != quorum.evidence_commitment() {
        return Err(ReviewBasisCurrentnessPolicyTrustError::ReviewBasisQuorumMismatch);
    }
    if currentness.observed_at_unix_ms() < quorum.quorum_observed_at_unix_ms() {
        return Err(ReviewBasisCurrentnessPolicyTrustError::ObservationBeforeQuorum);
    }

    let trust_algorithm = project_policy.authentication_provider_trust().algorithm();
    let provider_namespace =
        provider_namespace_commitment_v1(currentness.provider_namespace(), trust_algorithm)?;
    let verifier_identity =
        verifier_identity_commitment_v1(currentness.verifier(), trust_algorithm)?;

    if !provider_trust.trusts(&provider_namespace, &verifier_identity) {
        return Err(ReviewBasisCurrentnessPolicyTrustError::VerifierNotTrusted);
    }

    let provider_trust_policy = provider_trust
        .digest(trust_algorithm)
        .map_err(ReviewBasisCurrentnessPolicyTrustError::ProjectPolicy)?;

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
    push_digest(&mut out, currentness.evidence_commitment())?;
    push_digest(&mut out, &provider_namespace)?;
    push_digest(&mut out, &verifier_identity)?;
    push_digest(&mut out, &provider_trust_policy)?;
    out.extend_from_slice(&currentness.observed_at_unix_ms().to_be_bytes());
    push_digest(&mut out, currentness.coverage_commitment())?;

    Ok(ProjectPolicyTrustedReviewBasisCurrentnessV1 {
        project: quorum.project().clone(),
        proposal: quorum.proposal().clone(),
        authority_epoch: quorum.authority_epoch().clone(),
        project_policy: quorum.project_policy().clone(),
        repository_policy_state: quorum.repository_policy_state().clone(),
        review_basis_quorum: quorum.evidence_commitment().clone(),
        currentness_evidence: currentness.evidence_commitment().clone(),
        provider_namespace,
        verifier_identity,
        provider_trust_policy,
        currentness_observed_at_unix_ms: currentness.observed_at_unix_ms(),
        coverage_commitment: currentness.coverage_commitment().clone(),
        evidence_commitment: Digest::of_bytes(algorithm, &out),
    })
}

fn validate_string(
    value: &str,
    field: &'static str,
) -> Result<(), ReviewBasisCurrentnessPolicyTrustError> {
    if value.is_empty() || value.len() > MAX_STRING_LEN {
        return Err(ReviewBasisCurrentnessPolicyTrustError::InvalidString(field));
    }
    Ok(())
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewBasisCurrentnessPolicyTrustError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ReviewBasisCurrentnessPolicyTrustError> {
    push_string(out, digest.algorithm().id())?;
    let len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| ReviewBasisCurrentnessPolicyTrustError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
) -> Result<(), ReviewBasisCurrentnessPolicyTrustError> {
    let len = u32::try_from(value.len())
        .map_err(|_| ReviewBasisCurrentnessPolicyTrustError::CanonicalFieldTooLarge("string"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

/// Trust-join qualification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewBasisCurrentnessPolicyTrustError {
    /// The supplied project-policy/trust-policy state names another project.
    #[error("review-basis currentness trust project mismatch")]
    ProjectMismatch,
    /// The quorum's project-policy commitment does not match the supplied typed state.
    #[error("review-basis currentness trust project-policy mismatch")]
    ProjectPolicyMismatch,
    /// The typed project policy does not bind the supplied provider trust policy.
    #[error("review-basis currentness trust provider policy mismatch")]
    ProviderTrustPolicyMismatch,
    /// The currentness evidence names another protected review-basis quorum.
    #[error("review-basis currentness trust quorum mismatch")]
    ReviewBasisQuorumMismatch,
    /// The currentness observation predates the protected quorum.
    #[error("review-basis currentness trust observation predates quorum")]
    ObservationBeforeQuorum,
    /// The exact provider/verifier pair is not admitted by project policy.
    #[error("review-basis currentness verifier is not trusted by project policy")]
    VerifierNotTrusted,
    /// The provider/verifier identity input is empty or too large.
    #[error("review-basis currentness trust invalid {0}")]
    InvalidString(&'static str),
    /// Canonical evidence field exceeded its encoding bound.
    #[error("review-basis currentness trust canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
    /// Underlying project-policy canonicalization failed.
    #[error("review-basis currentness trust project-policy error: {0}")]
    ProjectPolicy(ProjectPolicyError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provider_namespace_commitment_is_domain_separated() {
        let a = provider_namespace_commitment_v1("provider-a", DigestAlgorithm::Sha256).unwrap();
        let b = verifier_identity_commitment_v1(
            &ReviewBasisCurrentnessVerifierIdentityV1::new("provider-a", "v1").unwrap(),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn provider_namespace_commitment_is_deterministic() {
        let a = provider_namespace_commitment_v1("provider-a", DigestAlgorithm::Sha256).unwrap();
        let b = provider_namespace_commitment_v1("provider-a", DigestAlgorithm::Sha256).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn invalid_namespace_fails_closed() {
        assert_eq!(
            provider_namespace_commitment_v1("", DigestAlgorithm::Sha256).unwrap_err(),
            ReviewBasisCurrentnessPolicyTrustError::InvalidString("provider namespace")
        );
    }

    #[test]
    fn invalid_verifier_identity_fails_closed() {
        let identity = ReviewBasisCurrentnessVerifierIdentityV1::new("", "v1").unwrap_err();
        assert!(identity.to_string().contains("verifier"));
    }
}

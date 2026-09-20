// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Project-policy trust binding for provider-verified Forge reviews.
//!
//! Provider cryptography answers whether an exact Xenia verifier identity
//! attested an exact authentication observation. Project policy answers a
//! different question: whether this project recognizes that verifier identity
//! as authority for the Xenia provider namespace.
//!
//! This crate joins those theorems without promoting caller-supplied time,
//! review eligibility, or provider provenance into quorum or merge authority.
//!
//! ```text
//! ProviderVerifiedEligibleReviewV1
//! + exact ChangeProposal
//! + exact ProjectPolicyStateV1 committed by that proposal
//! + exact AuthenticationProviderTrustPolicyV1 bound by that policy state
//! + exact (Xenia namespace, verifier identity) membership
//!     ↓
//! ProjectPolicyTrustedReviewV1
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, ProtocolVersion};
use mycelix_forge_project_policy::{
    AuthenticationProviderTrustPolicyV1, ProjectPolicyError, ProjectPolicyStateV1,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_provider::ProviderVerifiedEligibleReviewV1;
use mycelix_forge_xenia::xenia_provider_namespace;
use serde::Serialize;
use thiserror::Error;

const PROJECT_POLICY_TRUSTED_REVIEW_DOMAIN_V1: &[u8] =
    b"mycelix-forge/project-policy-trusted-review/v1\0";

/// Positive result proving that the exact provider verifier used by one
/// provider-verified eligible review is trusted by the exact project policy
/// committed into that review's immutable proposal.
///
/// This value remains intentionally weaker than quorum or merge authority. It
/// is serializable for evidence export but not deserializable into positive
/// authority; callers must re-run [`bind_project_policy_trusted_review_v1`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectPolicyTrustedReviewV1 {
    review: ProviderVerifiedEligibleReviewV1,
    proposal: ChangeProposalId,
    project_policy: Digest,
    provider_trust_policy: Digest,
    provider_namespace: Digest,
    verifier_identity: Digest,
    evidence_commitment: Digest,
}

impl ProjectPolicyTrustedReviewV1 {
    /// Provider-verified, structurally eligible review whose verifier trust
    /// has now been established from the proposal's exact project policy.
    pub fn review(&self) -> &ProviderVerifiedEligibleReviewV1 {
        &self.review
    }

    /// Exact immutable proposal subject.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact project-policy-state commitment carried by the proposal.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact authentication-provider trust-policy commitment bound by the
    /// project-policy state.
    pub fn provider_trust_policy(&self) -> &Digest {
        &self.provider_trust_policy
    }

    /// Exact provider namespace under which the verifier is trusted.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Exact verifier identity admitted by project policy.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Aggregate evidence commitment for this exact trust join.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one provider-verified review to the exact authentication-provider
/// trust policy committed by its immutable proposal.
pub fn bind_project_policy_trusted_review_v1(
    proposal: &ChangeProposal,
    project_policy: &ProjectPolicyStateV1,
    provider_trust: &AuthenticationProviderTrustPolicyV1,
    review: ProviderVerifiedEligibleReviewV1,
) -> Result<ProjectPolicyTrustedReviewV1, ProjectPolicyTrustedReviewError> {
    if project_policy.project() != proposal.project() || provider_trust.project() != proposal.project() {
        return Err(ProjectPolicyTrustedReviewError::ProjectMismatch);
    }

    let evidence_bound = review.review().review();
    if evidence_bound.project() != proposal.project() {
        return Err(ProjectPolicyTrustedReviewError::ProjectMismatch);
    }
    if evidence_bound.authority_epoch() != proposal.authority_epoch() {
        return Err(ProjectPolicyTrustedReviewError::AuthorityEpochMismatch);
    }

    let statement = evidence_bound.statement();
    let expected_proposal = proposal.proposal_id(statement.proposal().commitment().algorithm())?;
    if statement.proposal() != &expected_proposal {
        return Err(ProjectPolicyTrustedReviewError::ProposalMismatch);
    }

    let expected_project_policy = project_policy.digest(proposal.project_policy().algorithm())?;
    if proposal.project_policy() != &expected_project_policy {
        return Err(ProjectPolicyTrustedReviewError::ProjectPolicyMismatch);
    }

    if !project_policy.binds_provider_trust(provider_trust)? {
        return Err(ProjectPolicyTrustedReviewError::ProviderTrustPolicyMismatch);
    }

    let provider_namespace = xenia_provider_namespace();
    if !provider_trust.trusts(&provider_namespace, review.verifier_identity()) {
        return Err(ProjectPolicyTrustedReviewError::VerifierNotTrusted);
    }

    let trust_policy_digest =
        provider_trust.digest(project_policy.authentication_provider_trust().algorithm())?;
    let algorithm = review.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(PROJECT_POLICY_TRUSTED_REVIEW_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, review.evidence_commitment())?;
    push_digest(&mut out, expected_proposal.commitment())?;
    push_digest(&mut out, &expected_project_policy)?;
    push_digest(&mut out, &trust_policy_digest)?;
    push_digest(&mut out, &provider_namespace)?;
    push_digest(&mut out, review.verifier_identity())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProjectPolicyTrustedReviewV1 {
        verifier_identity: review.verifier_identity().clone(),
        review,
        proposal: expected_proposal,
        project_policy: expected_project_policy,
        provider_trust_policy: trust_policy_digest,
        provider_namespace,
        evidence_commitment,
    })
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ProjectPolicyTrustedReviewError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProjectPolicyTrustedReviewError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ProjectPolicyTrustedReviewError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Project-policy trust-join failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ProjectPolicyTrustedReviewError {
    /// Proposal, project-policy state, trust policy, or review belongs to a
    /// different project.
    #[error("project-policy trusted review project mismatch")]
    ProjectMismatch,
    /// Review authority epoch differs from the proposal's exact epoch.
    #[error("project-policy trusted review authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Review statement names another immutable proposal.
    #[error("project-policy trusted review proposal mismatch")]
    ProposalMismatch,
    /// Supplied project-policy state is not the exact state committed by the
    /// proposal.
    #[error("supplied project-policy state does not match proposal commitment")]
    ProjectPolicyMismatch,
    /// Supplied provider-trust policy is not the exact policy bound by the
    /// supplied project-policy state.
    #[error("provider-trust policy does not match project-policy state")]
    ProviderTrustPolicyMismatch,
    /// The exact Xenia provider namespace + verifier identity pair is not
    /// admitted by project policy.
    #[error("Xenia verifier identity is not trusted by project policy")]
    VerifierNotTrusted,
    /// Canonical evidence field exceeded the v1 length bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum length.
        max: usize,
    },
    /// Proposal canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Project-policy canonicalization failed.
    #[error(transparent)]
    ProjectPolicy(#[from] ProjectPolicyError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as _, SigningKey};
    use ml_dsa::{
        B32, MlDsa65, Signature as MlDsaSignature, SigningKey as MlDsaSigningKey,
        signature::{Keypair as _, Signer as _},
    };
    use mycelix_forge_authentication::{
        PrincipalAuthenticationRequest, PrincipalBinding, bind_principal_authentication,
    };
    use mycelix_forge_authority::{
        AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant, PrincipalId,
    };
    use mycelix_forge_core::{
        DigestAlgorithm, ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };
    use mycelix_forge_project_policy::TrustedProviderVerifierV1;
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review::{
        ReviewDecision, ReviewStatement, bind_review_evidence, qualify_review_eligibility,
    };
    use mycelix_forge_review_provider::bind_xenia_provider_verified_review_v1;
    use mycelix_forge_xenia::{
        XeniaHybridSuite, XeniaVerificationReceiptV1, xenia_challenge_commitment,
        xenia_key_lineage_commitment, xenia_operator_id_commitment,
    };
    use mycelix_forge_xenia_provider::{
        TrustedXeniaVerifierV1, XeniaProviderAttestedReceiptV1,
        provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
    };
    use serde::Serialize;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn authority(reviewer: PrincipalId) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(3_000),
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(reviewer, [Capability::ReviewSource]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    struct ProviderIdentity {
        ed: SigningKey,
        ml: MlDsaSigningKey<MlDsa65>,
    }

    impl ProviderIdentity {
        fn from_seeds(ed_seed: [u8; 32], ml_seed: [u8; 32]) -> Self {
            let ml_seed: B32 = ml_seed.into();
            Self {
                ed: SigningKey::from_bytes(&ed_seed),
                ml: MlDsaSigningKey::<MlDsa65>::from_seed(&ml_seed),
            }
        }

        fn trusted(&self) -> TrustedXeniaVerifierV1 {
            TrustedXeniaVerifierV1::new(
                self.ed.verifying_key().to_bytes(),
                self.ml.verifying_key().encode().as_slice().to_vec(),
            )
            .unwrap()
        }

        fn envelope(&self, receipt: XeniaVerificationReceiptV1) -> XeniaProviderAttestedReceiptV1 {
            #[derive(Serialize)]
            struct WireEnvelope {
                receipt: XeniaVerificationReceiptV1,
                verifier_identity_commitment: Digest,
                ed25519_signature: Vec<u8>,
                ml_dsa_65_signature: Vec<u8>,
            }

            let trusted = self.trusted();
            let transcript = provider_attestation_transcript_v1(
                &receipt,
                trusted.identity_commitment(),
            )
            .unwrap();
            let ml_signature: MlDsaSignature<MlDsa65> = self.ml.sign(&transcript);
            let wire = WireEnvelope {
                receipt,
                verifier_identity_commitment: trusted.identity_commitment().clone(),
                ed25519_signature: self.ed.sign(&transcript).to_bytes().to_vec(),
                ml_dsa_65_signature: ml_signature.encode().as_slice().to_vec(),
            };
            serde_json::from_slice(&serde_json::to_vec(&wire).unwrap()).unwrap()
        }
    }

    struct Fixture {
        proposal: ChangeProposal,
        project_policy: ProjectPolicyStateV1,
        provider_trust: AuthenticationProviderTrustPolicyV1,
        review: ProviderVerifiedEligibleReviewV1,
    }

    fn fixture(
        provider: &ProviderIdentity,
        trust_namespace: Digest,
        trust_verifier: Digest,
    ) -> Fixture {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone());
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(trust_namespace, trust_verifier)],
        )
        .unwrap();
        let project_policy = ProjectPolicyStateV1::new(
            project(),
            0,
            None,
            &provider_trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let repository_policy =
            RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        let proposal = ChangeProposal::new(
            principal(0x30),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let statement = ReviewStatement::new(
            &proposal,
            reviewer.clone(),
            ReviewDecision::Approve,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();

        let operator = xenia_operator_id_commitment("operator:alice").unwrap();
        let lineage = xenia_key_lineage_commitment(&[0x21; 32], &[0x22; 64], None).unwrap();
        let binding = PrincipalBinding::new(
            xenia_provider_namespace(),
            operator.clone(),
            reviewer.clone(),
            lineage.clone(),
        );
        let request = PrincipalAuthenticationRequest::new(
            project(),
            authority.digest(DigestAlgorithm::Sha256).unwrap(),
            reviewer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            statement_id.commitment().clone(),
            [0x80; 32],
        );
        let receipt = XeniaVerificationReceiptV1::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            operator,
            lineage,
            xenia_challenge_commitment(request.challenge()),
            XeniaHybridSuite::Ed25519MlDsa65V1,
            digest(0x71),
            digest(0x72),
            digest(0x73),
            1_797_000_000,
        );
        let trusted = provider.trusted();
        let envelope = provider.envelope(receipt);
        let provider_verified = verify_xenia_provider_authentication_v1(
            &trusted,
            &request,
            &binding,
            &envelope,
        )
        .unwrap();
        let authentication = bind_principal_authentication(
            request,
            binding,
            &authority,
            provider_verified.observation().clone(),
        )
        .unwrap();
        let evidence_bound =
            bind_review_evidence(&proposal, statement, &authentication, &authority).unwrap();
        let eligible = qualify_review_eligibility(evidence_bound, &authority, 1_500).unwrap();
        let review = bind_xenia_provider_verified_review_v1(
            eligible,
            &authentication,
            &provider_verified,
        )
        .unwrap();

        Fixture {
            proposal,
            project_policy,
            provider_trust,
            review,
        }
    }

    #[test]
    fn exact_project_policy_trust_promotes_provider_verified_review() {
        let provider = ProviderIdentity::from_seeds([0x81; 32], [0x82; 32]);
        let trusted = provider.trusted();
        let fixture = fixture(
            &provider,
            xenia_provider_namespace(),
            trusted.identity_commitment().clone(),
        );

        let positive = bind_project_policy_trusted_review_v1(
            &fixture.proposal,
            &fixture.project_policy,
            &fixture.provider_trust,
            fixture.review,
        )
        .unwrap();
        assert_eq!(positive.verifier_identity(), trusted.identity_commitment());
        assert_eq!(positive.provider_namespace(), &xenia_provider_namespace());
    }

    #[test]
    fn same_verifier_under_wrong_provider_namespace_is_not_trusted() {
        let provider = ProviderIdentity::from_seeds([0x83; 32], [0x84; 32]);
        let trusted = provider.trusted();
        let fixture = fixture(
            &provider,
            digest(0xa0),
            trusted.identity_commitment().clone(),
        );

        assert_eq!(
            bind_project_policy_trusted_review_v1(
                &fixture.proposal,
                &fixture.project_policy,
                &fixture.provider_trust,
                fixture.review,
            )
            .unwrap_err(),
            ProjectPolicyTrustedReviewError::VerifierNotTrusted
        );
    }

    #[test]
    fn untrusted_verifier_identity_is_not_promoted() {
        let provider = ProviderIdentity::from_seeds([0x85; 32], [0x86; 32]);
        let fixture = fixture(&provider, xenia_provider_namespace(), digest(0xa1));

        assert_eq!(
            bind_project_policy_trusted_review_v1(
                &fixture.proposal,
                &fixture.project_policy,
                &fixture.provider_trust,
                fixture.review,
            )
            .unwrap_err(),
            ProjectPolicyTrustedReviewError::VerifierNotTrusted
        );
    }

    #[test]
    fn trust_policy_not_bound_by_project_policy_is_rejected() {
        let provider = ProviderIdentity::from_seeds([0x87; 32], [0x88; 32]);
        let trusted = provider.trusted();
        let fixture = fixture(
            &provider,
            xenia_provider_namespace(),
            trusted.identity_commitment().clone(),
        );
        let foreign_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(
                xenia_provider_namespace(),
                digest(0xa2),
            )],
        )
        .unwrap();

        assert_eq!(
            bind_project_policy_trusted_review_v1(
                &fixture.proposal,
                &fixture.project_policy,
                &foreign_trust,
                fixture.review,
            )
            .unwrap_err(),
            ProjectPolicyTrustedReviewError::ProviderTrustPolicyMismatch
        );
    }

    #[test]
    fn verifier_trust_rotation_changes_proposal_and_invalidates_old_review() {
        let provider = ProviderIdentity::from_seeds([0x89; 32], [0x8a; 32]);
        let trusted = provider.trusted();
        let old = fixture(
            &provider,
            xenia_provider_namespace(),
            trusted.identity_commitment().clone(),
        );

        let rotated_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(
                xenia_provider_namespace(),
                digest(0xa3),
            )],
        )
        .unwrap();
        let rotated_policy = ProjectPolicyStateV1::new(
            project(),
            0,
            None,
            &rotated_trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let repository_policy =
            RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        let rotated_proposal = ChangeProposal::new(
            principal(0x30),
            &authority(principal(0x20)),
            &rotated_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();

        assert_eq!(
            bind_project_policy_trusted_review_v1(
                &rotated_proposal,
                &rotated_policy,
                &rotated_trust,
                old.review,
            )
            .unwrap_err(),
            ProjectPolicyTrustedReviewError::ProposalMismatch
        );
    }
}

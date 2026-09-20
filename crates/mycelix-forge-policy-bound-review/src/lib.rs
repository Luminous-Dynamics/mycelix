// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bind project-policy-trusted review revisions to the exact review-policy
//! context accepted by project policy authority.
//!
//! ```text
//! ProposalReviewPolicyAuthorityQuorumV1
//! + ProjectPolicyTrustedReviewRevisionV1
//! + exact ProposalReviewPolicyContextV1
//! + review_context == adopted policy-context digest
//! + policy adoption observation <= review observation
//!     ↓
//! PolicyAdoptionBoundTrustedReviewRevisionV1
//! ```
//!
//! This closes the substitution gap where a project could authorize review
//! policy A while a reviewer authenticated a revision carrying arbitrary
//! context B. The time comparison is structural only: both observations remain
//! caller-supplied and this crate does not establish trusted wall-clock time.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ProposalError};
use mycelix_forge_review_acceptance_policy::{
    ProposalReviewPolicyContextV1, ReviewAcceptancePolicyError, ReviewAcceptancePolicyV1,
};
use mycelix_forge_review_policy_adoption::ProposalReviewPolicyAuthorityQuorumV1;
use mycelix_forge_review_state::{ReviewRevisionV1, ReviewStateError};
use mycelix_forge_review_state_auth::ProjectPolicyTrustedReviewRevisionV1;
use serde::Serialize;
use thiserror::Error;

const POLICY_BOUND_REVIEW_DOMAIN_V1: &[u8] =
    b"mycelix-forge/policy-adoption-bound-trusted-review/v1\0";

/// Positive result proving that one exact project-policy-trusted review revision
/// authenticated the exact review context accepted by the proposal's project-
/// policy review-policy quorum.
///
/// The nested trusted revision is retained rather than flattened so downstream
/// code cannot reconstruct a weaker approximation of the provider/authentication
/// theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PolicyAdoptionBoundTrustedReviewRevisionV1 {
    revision: ProjectPolicyTrustedReviewRevisionV1,
    review_policy: Digest,
    review_policy_context: Digest,
    policy_authority_evidence: Digest,
    policy_adopted_at_unix_ms: u64,
    review_observed_at_unix_ms: u64,
    evidence_commitment: Digest,
}

impl PolicyAdoptionBoundTrustedReviewRevisionV1 {
    /// Exact project-policy-trusted authenticated review revision.
    pub fn revision(&self) -> &ProjectPolicyTrustedReviewRevisionV1 {
        &self.revision
    }

    /// Exact raw review revision at the bottom of the trusted-review theorem.
    pub fn exact_review_revision(&self) -> &ReviewRevisionV1 {
        self.revision
            .revision()
            .revision()
            .revision()
            .revision()
    }

    /// Exact project-authorized review-policy commitment.
    pub fn review_policy(&self) -> &Digest {
        &self.review_policy
    }

    /// Exact adopted proposal-review policy-context commitment.
    pub fn review_policy_context(&self) -> &Digest {
        &self.review_policy_context
    }

    /// Exact project-policy quorum evidence accepting the policy.
    pub fn policy_authority_evidence(&self) -> &Digest {
        &self.policy_authority_evidence
    }

    /// Caller-supplied common observation time of the policy-authority quorum.
    pub const fn policy_adopted_at_unix_ms(&self) -> u64 {
        self.policy_adopted_at_unix_ms
    }

    /// Caller-supplied observation time used for structural review eligibility.
    pub const fn review_observed_at_unix_ms(&self) -> u64 {
        self.review_observed_at_unix_ms
    }

    /// Aggregate policy-bound review evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind an already project-policy-trusted authenticated review revision to the
/// exact proposal review-policy context accepted by project policy authority.
pub fn bind_policy_adoption_to_trusted_review_revision_v1(
    proposal: &ChangeProposal,
    policy: &ReviewAcceptancePolicyV1,
    policy_context: &ProposalReviewPolicyContextV1,
    policy_authority: &ProposalReviewPolicyAuthorityQuorumV1,
    trusted_review: ProjectPolicyTrustedReviewRevisionV1,
) -> Result<PolicyAdoptionBoundTrustedReviewRevisionV1, PolicyBoundReviewError> {
    if policy.project() != proposal.project()
        || policy_context.project() != proposal.project()
        || policy_authority.project() != proposal.project()
    {
        return Err(PolicyBoundReviewError::ProjectMismatch);
    }

    let expected_context_proposal =
        proposal.proposal_id(policy_context.proposal().commitment().algorithm())?;
    if policy_context.proposal() != &expected_context_proposal {
        return Err(PolicyBoundReviewError::ProposalMismatch);
    }
    let expected_context_policy =
        policy.digest(policy_context.review_acceptance_policy().algorithm())?;
    if policy_context.review_acceptance_policy() != &expected_context_policy {
        return Err(PolicyBoundReviewError::ReviewPolicyMismatch);
    }

    let expected_authority_proposal =
        proposal.proposal_id(policy_authority.proposal().commitment().algorithm())?;
    if policy_authority.proposal() != &expected_authority_proposal {
        return Err(PolicyBoundReviewError::ProposalMismatch);
    }
    let expected_authority_policy = policy.digest(policy_authority.review_policy().algorithm())?;
    if policy_authority.review_policy() != &expected_authority_policy {
        return Err(PolicyBoundReviewError::ReviewPolicyMismatch);
    }
    let expected_authority_context =
        policy_context.digest(policy_authority.review_policy_context().algorithm())?;
    if policy_authority.review_policy_context() != &expected_authority_context {
        return Err(PolicyBoundReviewError::ReviewPolicyContextMismatch);
    }
    if policy_authority.authority_epoch() != proposal.authority_epoch() {
        return Err(PolicyBoundReviewError::AuthorityEpochMismatch);
    }
    if policy_authority.project_policy() != proposal.project_policy() {
        return Err(PolicyBoundReviewError::ProjectPolicyMismatch);
    }
    if trusted_review.project_policy() != proposal.project_policy() {
        return Err(PolicyBoundReviewError::ProjectPolicyMismatch);
    }

    let provider_verified = trusted_review.revision();
    let eligible = provider_verified.revision();
    let evidence_bound = eligible.revision();
    let exact_revision = evidence_bound.revision();

    if evidence_bound.project() != proposal.project() {
        return Err(PolicyBoundReviewError::ProjectMismatch);
    }
    if evidence_bound.authority_epoch() != proposal.authority_epoch() {
        return Err(PolicyBoundReviewError::AuthorityEpochMismatch);
    }
    let expected_review_proposal =
        proposal.proposal_id(exact_revision.proposal().commitment().algorithm())?;
    if exact_revision.proposal() != &expected_review_proposal {
        return Err(PolicyBoundReviewError::ProposalMismatch);
    }

    let expected_review_context = policy_context.digest(exact_revision.review_context().algorithm())?;
    if exact_revision.review_context() != &expected_review_context {
        return Err(PolicyBoundReviewError::ReviewContextMismatch);
    }

    if policy_authority.quorum_observed_at_unix_ms() > eligible.observed_at_unix_ms() {
        return Err(PolicyBoundReviewError::ReviewObservedBeforePolicyAdoption {
            policy_adopted_at_unix_ms: policy_authority.quorum_observed_at_unix_ms(),
            review_observed_at_unix_ms: eligible.observed_at_unix_ms(),
        });
    }

    let algorithm = trusted_review.evidence_commitment().algorithm();
    let canonical_revision = exact_revision.revision_id(algorithm)?;
    let policy_digest = policy.digest(algorithm)?;
    let context_digest = policy_context.digest(algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(POLICY_BOUND_REVIEW_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, expected_review_proposal.commitment())?;
    push_digest(&mut out, canonical_revision.commitment())?;
    push_digest(&mut out, &policy_digest)?;
    push_digest(&mut out, &context_digest)?;
    push_digest(&mut out, policy_authority.evidence_commitment())?;
    push_digest(&mut out, trusted_review.evidence_commitment())?;
    out.extend_from_slice(&policy_authority.quorum_observed_at_unix_ms().to_be_bytes());
    out.extend_from_slice(&eligible.observed_at_unix_ms().to_be_bytes());
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(PolicyAdoptionBoundTrustedReviewRevisionV1 {
        revision: trusted_review,
        review_policy: policy_digest,
        review_policy_context: context_digest,
        policy_authority_evidence: policy_authority.evidence_commitment().clone(),
        policy_adopted_at_unix_ms: policy_authority.quorum_observed_at_unix_ms(),
        review_observed_at_unix_ms: eligible.observed_at_unix_ms(),
        evidence_commitment,
    })
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), PolicyBoundReviewError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), PolicyBoundReviewError> {
    let len = u32::try_from(bytes.len()).map_err(|_| PolicyBoundReviewError::CanonicalFieldTooLarge {
        field,
        len: bytes.len(),
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Policy-adoption/trusted-review join failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum PolicyBoundReviewError {
    /// Proposal, policy, policy authority, or review belongs to another project.
    #[error("policy-bound trusted review project mismatch")]
    ProjectMismatch,
    /// Policy context, adoption quorum, or review names another proposal.
    #[error("policy-bound trusted review proposal mismatch")]
    ProposalMismatch,
    /// Policy context or adoption quorum names another review policy.
    #[error("policy-bound trusted review policy mismatch")]
    ReviewPolicyMismatch,
    /// Project-policy adoption names another proposal-review policy context.
    #[error("policy-bound trusted review policy-context mismatch")]
    ReviewPolicyContextMismatch,
    /// Review revision did not authenticate the exact adopted review context.
    #[error("review revision context does not match adopted proposal-review policy context")]
    ReviewContextMismatch,
    /// Review/adoption authority epoch differs from the proposal's exact epoch.
    #[error("policy-bound trusted review authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Review/adoption project policy differs from the proposal's exact policy.
    #[error("policy-bound trusted review project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Structural review observation predates structural policy-adoption quorum.
    #[error(
        "review observed before policy adoption: review={review_observed_at_unix_ms}, adoption={policy_adopted_at_unix_ms}"
    )]
    ReviewObservedBeforePolicyAdoption {
        /// Caller-supplied common policy-adoption observation time.
        policy_adopted_at_unix_ms: u64,
        /// Caller-supplied structural review observation time.
        review_observed_at_unix_ms: u64,
    },
    /// Canonical field exceeded v1 encoding bounds.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum length.
        max: usize,
    },
    /// Proposal identity/canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Review-policy validation/canonicalization failed.
    #[error(transparent)]
    ReviewPolicy(#[from] ReviewAcceptancePolicyError),
    /// Review-revision canonicalization failed.
    #[error(transparent)]
    ReviewState(#[from] ReviewStateError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as _, SigningKey};
    use ml_dsa::{
        signature::{Keypair as _, Signer as _}, B32, MlDsa65,
        Signature as MlDsaSignature, SigningKey as MlDsaSigningKey,
    };
    use mycelix_forge_authentication::{
        bind_principal_authentication, EvidenceBoundPrincipalAuthentication,
        PrincipalAuthenticationRequest, PrincipalBinding,
    };
    use mycelix_forge_authentication_policy::bind_project_policy_trusted_xenia_authentication_v1;
    use mycelix_forge_authority::{
        AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant, PrincipalId,
    };
    use mycelix_forge_core::{
        DigestAlgorithm, ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review_acceptance_policy::{
        ProposerReviewRuleV1, RequestChangesRuleV1,
    };
    use mycelix_forge_review_policy_adoption::{
        bind_review_policy_authority_attestation_v1,
        evaluate_review_policy_authority_quorum_v1,
        ProposalReviewPolicyAdoptionStatementV1, ReviewPolicyAuthorityAttestationV1,
    };
    use mycelix_forge_review_state::ReviewRevisionDecision;
    use mycelix_forge_review_state_auth::{
        bind_project_policy_trusted_review_revision_v1,
        bind_review_revision_authentication_v1,
        bind_xenia_provider_verified_review_revision_v1,
        qualify_review_revision_eligibility_v1,
    };
    use mycelix_forge_xenia::{
        xenia_challenge_commitment, xenia_key_lineage_commitment,
        xenia_operator_id_commitment, xenia_provider_namespace, XeniaHybridSuite,
        XeniaVerificationReceiptV1,
    };
    use mycelix_forge_xenia_provider::{
        provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
        ProviderVerifiedXeniaAuthenticationV1, TrustedXeniaVerifierV1,
        XeniaProviderAttestedReceiptV1,
    };
    use serde::Serialize;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    struct ProviderIdentity {
        ed: SigningKey,
        ml: MlDsaSigningKey<MlDsa65>,
    }

    impl ProviderIdentity {
        fn new() -> Self {
            let seed: B32 = [0x82; 32].into();
            Self {
                ed: SigningKey::from_bytes(&[0x81; 32]),
                ml: MlDsaSigningKey::<MlDsa65>::from_seed(&seed),
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
            serde_json::from_slice(
                &serde_json::to_vec(&WireEnvelope {
                    receipt,
                    verifier_identity_commitment: trusted.identity_commitment().clone(),
                    ed25519_signature: self.ed.sign(&transcript).to_bytes().to_vec(),
                    ml_dsa_65_signature: ml_signature.encode().as_slice().to_vec(),
                })
                .unwrap(),
            )
            .unwrap()
        }
    }

    struct Context {
        provider: ProviderIdentity,
        authority: AuthorityEpoch,
        project_policy: ProjectPolicyStateV1,
        provider_trust: AuthenticationProviderTrustPolicyV1,
        proposal: ChangeProposal,
        policy: ReviewAcceptancePolicyV1,
        policy_context: ProposalReviewPolicyContextV1,
    }

    fn context() -> Context {
        let provider = ProviderIdentity::new();
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(
                xenia_provider_namespace(),
                provider.trusted().identity_commitment().clone(),
            )],
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
        let authority = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(5_000),
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(principal(0x20), [Capability::ManagePolicy]).unwrap(),
                PrincipalGrant::new(principal(0x21), [Capability::ManagePolicy]).unwrap(),
                PrincipalGrant::new(principal(0x40), [Capability::ReviewSource]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ManagePolicy, 2).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap();
        let repository_policy =
            RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        let proposal = ChangeProposal::new(
            principal(0x30),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x40; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x41; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x42; 20]).unwrap(),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let policy = ReviewAcceptancePolicyV1::new(
            project(),
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![principal(0x40)],
        )
        .unwrap();
        let policy_context = ProposalReviewPolicyContextV1::new(
            &proposal,
            &policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        Context {
            provider,
            authority,
            project_policy,
            provider_trust,
            proposal,
            policy,
            policy_context,
        }
    }

    struct RawAuth {
        authentication: EvidenceBoundPrincipalAuthentication,
        provider_verified: ProviderVerifiedXeniaAuthenticationV1,
    }

    fn raw_auth(
        context: &Context,
        signer: PrincipalId,
        action_subject: Digest,
        capability: Capability,
        marker: u8,
    ) -> RawAuth {
        let operator = xenia_operator_id_commitment(&format!("operator:bound-review-{marker}"))
            .unwrap();
        let lineage = xenia_key_lineage_commitment(
            &[marker.wrapping_add(1); 32],
            &[marker.wrapping_add(2); 64],
            None,
        )
        .unwrap();
        let binding = PrincipalBinding::new(
            xenia_provider_namespace(),
            operator.clone(),
            signer.clone(),
            lineage.clone(),
        );
        let request = PrincipalAuthenticationRequest::new(
            project(),
            context.authority.digest(DigestAlgorithm::Sha256).unwrap(),
            signer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            capability,
            action_subject,
            [marker; 32],
        );
        let receipt = XeniaVerificationReceiptV1::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            operator,
            lineage,
            xenia_challenge_commitment(request.challenge()),
            XeniaHybridSuite::Ed25519MlDsa65V1,
            digest(marker.wrapping_add(3)),
            digest(marker.wrapping_add(4)),
            digest(marker.wrapping_add(5)),
            1_797_000_000,
        );
        let trusted_verifier = context.provider.trusted();
        let envelope = context.provider.envelope(receipt);
        let provider_verified = verify_xenia_provider_authentication_v1(
            &trusted_verifier,
            &request,
            &binding,
            &envelope,
        )
        .unwrap();
        let authentication = bind_principal_authentication(
            request,
            binding,
            &context.authority,
            provider_verified.observation().clone(),
        )
        .unwrap();
        RawAuth {
            authentication,
            provider_verified,
        }
    }

    fn trusted_auth(
        context: &Context,
        signer: PrincipalId,
        action_subject: Digest,
        capability: Capability,
        marker: u8,
    ) -> mycelix_forge_authentication_policy::ProjectPolicyTrustedXeniaAuthenticationV1 {
        let raw = raw_auth(context, signer, action_subject, capability, marker);
        bind_project_policy_trusted_xenia_authentication_v1(
            &context.proposal,
            &context.project_policy,
            &context.provider_trust,
            &raw.authentication,
            &raw.provider_verified,
        )
        .unwrap()
    }

    fn policy_authority(
        context: &Context,
        adopted_at: u64,
    ) -> ProposalReviewPolicyAuthorityQuorumV1 {
        let statement = ProposalReviewPolicyAdoptionStatementV1::new(
            &context.proposal,
            &context.policy,
            &context.policy_context,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let trusted_a = trusted_auth(
            context,
            principal(0x20),
            statement_id.commitment().clone(),
            Capability::ManagePolicy,
            0x80,
        );
        let trusted_b = trusted_auth(
            context,
            principal(0x21),
            statement_id.commitment().clone(),
            Capability::ManagePolicy,
            0x81,
        );
        let a: ReviewPolicyAuthorityAttestationV1 =
            bind_review_policy_authority_attestation_v1(
                &context.proposal,
                &context.policy,
                &context.policy_context,
                &statement,
                &trusted_a,
                &context.authority,
                adopted_at.saturating_sub(100),
            )
            .unwrap();
        let b: ReviewPolicyAuthorityAttestationV1 =
            bind_review_policy_authority_attestation_v1(
                &context.proposal,
                &context.policy,
                &context.policy_context,
                &statement,
                &trusted_b,
                &context.authority,
                adopted_at.saturating_sub(50),
            )
            .unwrap();
        evaluate_review_policy_authority_quorum_v1(
            &context.proposal,
            &context.policy,
            &context.policy_context,
            &statement,
            &context.authority,
            &[a, b],
            adopted_at,
        )
        .unwrap()
    }

    fn trusted_review_with_context(
        context: &Context,
        review_context: Digest,
        observed_at: u64,
        marker: u8,
    ) -> ProjectPolicyTrustedReviewRevisionV1 {
        let revision = ReviewRevisionV1::genesis(
            &context.proposal,
            principal(0x40),
            ReviewRevisionDecision::Approve,
            review_context,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let revision_id = revision.revision_id(DigestAlgorithm::Sha256).unwrap();
        let raw = raw_auth(
            context,
            principal(0x40),
            revision_id.commitment().clone(),
            Capability::ReviewSource,
            marker,
        );
        let evidence_bound = bind_review_revision_authentication_v1(
            &context.proposal,
            revision,
            &raw.authentication,
            &context.authority,
        )
        .unwrap();
        let eligible = qualify_review_revision_eligibility_v1(
            evidence_bound,
            &context.authority,
            observed_at,
        )
        .unwrap();
        let provider_bound = bind_xenia_provider_verified_review_revision_v1(
            eligible,
            &raw.authentication,
            &raw.provider_verified,
        )
        .unwrap();
        bind_project_policy_trusted_review_revision_v1(
            &context.proposal,
            &context.project_policy,
            &context.provider_trust,
            provider_bound,
        )
        .unwrap()
    }

    #[test]
    fn exact_adopted_context_binds_trusted_review() {
        let context = context();
        let authority = policy_authority(&context, 1_500);
        let review_context = context
            .policy_context
            .digest(DigestAlgorithm::Sha256)
            .unwrap();
        let trusted_review = trusted_review_with_context(&context, review_context, 1_700, 0x90);
        let result = bind_policy_adoption_to_trusted_review_revision_v1(
            &context.proposal,
            &context.policy,
            &context.policy_context,
            &authority,
            trusted_review,
        )
        .unwrap();
        assert_eq!(result.policy_adopted_at_unix_ms(), 1_500);
        assert_eq!(result.review_observed_at_unix_ms(), 1_700);
        assert_eq!(result.exact_review_revision().reviewer(), &principal(0x40));
    }

    #[test]
    fn arbitrary_authenticated_review_context_cannot_borrow_policy_adoption() {
        let context = context();
        let authority = policy_authority(&context, 1_500);
        let trusted_review = trusted_review_with_context(&context, digest(0xee), 1_700, 0x91);
        assert_eq!(
            bind_policy_adoption_to_trusted_review_revision_v1(
                &context.proposal,
                &context.policy,
                &context.policy_context,
                &authority,
                trusted_review,
            )
            .unwrap_err(),
            PolicyBoundReviewError::ReviewContextMismatch
        );
    }

    #[test]
    fn review_cannot_predate_structural_policy_adoption() {
        let context = context();
        let authority = policy_authority(&context, 1_500);
        let review_context = context
            .policy_context
            .digest(DigestAlgorithm::Sha256)
            .unwrap();
        let trusted_review = trusted_review_with_context(&context, review_context, 1_400, 0x92);
        assert_eq!(
            bind_policy_adoption_to_trusted_review_revision_v1(
                &context.proposal,
                &context.policy,
                &context.policy_context,
                &authority,
                trusted_review,
            )
            .unwrap_err(),
            PolicyBoundReviewError::ReviewObservedBeforePolicyAdoption {
                policy_adopted_at_unix_ms: 1_500,
                review_observed_at_unix_ms: 1_400,
            }
        );
    }

    #[test]
    fn changed_policy_cannot_reinterpret_old_authenticated_review() {
        let context = context();
        let authority = policy_authority(&context, 1_500);
        let review_context = context
            .policy_context
            .digest(DigestAlgorithm::Sha256)
            .unwrap();
        let trusted_review = trusted_review_with_context(&context, review_context, 1_700, 0x93);
        let changed_policy = ReviewAcceptancePolicyV1::new(
            project(),
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![principal(0x40)],
        )
        .unwrap();
        let changed_context = ProposalReviewPolicyContextV1::new(
            &context.proposal,
            &changed_policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_eq!(
            bind_policy_adoption_to_trusted_review_revision_v1(
                &context.proposal,
                &changed_policy,
                &changed_context,
                &authority,
                trusted_review,
            )
            .unwrap_err(),
            PolicyBoundReviewError::ReviewPolicyMismatch
        );
    }
}

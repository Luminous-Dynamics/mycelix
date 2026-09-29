// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Distinct approval quorum over project-policy-trusted Forge reviews.
//!
//! This crate deliberately accepts only [`ProjectPolicyTrustedReviewV1`]
//! values. A raw structural review, a provider-verified review whose verifier
//! is not admitted by project policy, or a review from another project-policy
//! state is therefore not representable as a quorum input.
//!
//! The quorum theorem is observation-time scoped:
//!
//! ```text
//! distinct ProjectPolicyTrustedReviewV1 approvals
//! + exact same ChangeProposalId
//! + exact same AuthorityEpoch
//! + exact proposal project/repository/project-policy context
//! + every review observed no later than one common quorum observation time
//! + every reviewer eligible for ReviewSource at that common time
//! + exact ReviewSource threshold
//!     ↓
//! ProjectPolicyReviewQuorumV1
//! ```
//!
//! The common time is still caller supplied. This crate does not establish a
//! trusted clock, absence of later/opposing reviews, repository correctness,
//! CI/build qualification, or merge authorization.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use std::collections::BTreeSet;

use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review::ReviewDecision;
use mycelix_forge_review_policy::ProjectPolicyTrustedReviewV1;
use serde::Serialize;
use thiserror::Error;

const REVIEW_QUORUM_DOMAIN_V1: &[u8] = b"mycelix-forge/project-policy-review-quorum/v1\0";

/// One normalized approval counted by a project-policy-trusted review quorum.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QuorumApprovalV1 {
    reviewer: PrincipalId,
    review_observed_at_unix_ms: u64,
    trusted_review_evidence: Digest,
    verifier_identity: Digest,
}

impl QuorumApprovalV1 {
    /// Distinct Forge reviewer principal counted by this quorum.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Caller-supplied observation time already bound by the review's
    /// structural-eligibility theorem.
    pub const fn review_observed_at_unix_ms(&self) -> u64 {
        self.review_observed_at_unix_ms
    }

    /// Exact project-policy-trusted review evidence commitment.
    pub fn trusted_review_evidence(&self) -> &Digest {
        &self.trusted_review_evidence
    }

    /// Exact provider verifier identity whose project trust was established for
    /// this review.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }
}

/// Positive result proving that one exact proposal has enough distinct,
/// project-policy-trusted `Approve` reviews at one common supplied observation
/// time to satisfy its exact `ReviewSource` threshold.
///
/// The type is serializable for evidence export but intentionally not
/// deserializable into positive authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectPolicyReviewQuorumV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    quorum_observed_at_unix_ms: u64,
    threshold: u16,
    approvals: Vec<QuorumApprovalV1>,
    evidence_commitment: Digest,
}

impl ProjectPolicyReviewQuorumV1 {
    /// Project containing the exact proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal subject approved by the quorum input set.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch used for threshold and eligibility evaluation.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact typed project-policy-state commitment carried by the proposal.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy-state commitment carried by the proposal.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Common caller-supplied time at which all counted reviewers were
    /// re-evaluated for `ReviewSource` eligibility.
    pub const fn quorum_observed_at_unix_ms(&self) -> u64 {
        self.quorum_observed_at_unix_ms
    }

    /// Exact `ReviewSource` threshold from the bound authority epoch.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    /// Canonically reviewer-sorted distinct approvals.
    pub fn approvals(&self) -> &[QuorumApprovalV1] {
        &self.approvals
    }

    /// Aggregate evidence commitment for this exact quorum input set and
    /// observation time.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate a distinct approval quorum from project-policy-trusted reviews.
pub fn evaluate_project_policy_review_quorum_v1(
    proposal: &ChangeProposal,
    authority_epoch: &AuthorityEpoch,
    quorum_observed_at_unix_ms: u64,
    reviews: &[ProjectPolicyTrustedReviewV1],
) -> Result<ProjectPolicyReviewQuorumV1, ReviewQuorumError> {
    if authority_epoch.project() != proposal.project() {
        return Err(ReviewQuorumError::ProjectMismatch);
    }

    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch {
        return Err(ReviewQuorumError::AuthorityEpochMismatch);
    }
    if !authority_epoch.is_valid_at(quorum_observed_at_unix_ms) {
        return Err(ReviewQuorumError::AuthorityEpochNotValidAtQuorumTime);
    }

    let threshold = authority_epoch
        .threshold_for(Capability::ReviewSource)
        .ok_or(ReviewQuorumError::MissingReviewThreshold)?;

    if reviews.is_empty() {
        return Err(ReviewQuorumError::UnderThreshold {
            required: threshold,
            approvals: 0,
        });
    }

    let proposal_algorithm = reviews[0].proposal().commitment().algorithm();
    let expected_proposal = proposal.proposal_id(proposal_algorithm)?;
    let mut seen = BTreeSet::new();
    let mut approvals = Vec::with_capacity(reviews.len());

    for review in reviews {
        if review.proposal() != &expected_proposal {
            return Err(ReviewQuorumError::ProposalMismatch);
        }
        if review.project_policy() != proposal.project_policy() {
            return Err(ReviewQuorumError::ProjectPolicyMismatch);
        }

        let provider_review = review.review();
        let structural_review = provider_review.review();
        let evidence_bound = structural_review.review();
        let statement = evidence_bound.statement();

        if evidence_bound.project() != proposal.project() {
            return Err(ReviewQuorumError::ProjectMismatch);
        }
        if evidence_bound.authority_epoch() != &expected_epoch {
            return Err(ReviewQuorumError::AuthorityEpochMismatch);
        }
        if statement.proposal() != &expected_proposal {
            return Err(ReviewQuorumError::ProposalMismatch);
        }
        if statement.decision() != ReviewDecision::Approve {
            return Err(ReviewQuorumError::NonApprovalDecision(
                statement.reviewer().clone(),
            ));
        }
        if structural_review.observed_at_unix_ms() > quorum_observed_at_unix_ms {
            return Err(ReviewQuorumError::ReviewObservedAfterQuorumTime(
                statement.reviewer().clone(),
            ));
        }
        if !authority_epoch.is_principal_eligible(
            statement.reviewer(),
            Capability::ReviewSource,
            quorum_observed_at_unix_ms,
        ) {
            return Err(ReviewQuorumError::ReviewerNotEligibleAtQuorumTime(
                statement.reviewer().clone(),
            ));
        }
        if !seen.insert(statement.reviewer().clone()) {
            return Err(ReviewQuorumError::DuplicateReviewer(
                statement.reviewer().clone(),
            ));
        }

        approvals.push(QuorumApprovalV1 {
            reviewer: statement.reviewer().clone(),
            review_observed_at_unix_ms: structural_review.observed_at_unix_ms(),
            trusted_review_evidence: review.evidence_commitment().clone(),
            verifier_identity: review.verifier_identity().clone(),
        });
    }

    approvals.sort_by(|a, b| a.reviewer.cmp(&b.reviewer));

    if approvals.len() < usize::from(threshold) {
        return Err(ReviewQuorumError::UnderThreshold {
            required: threshold,
            approvals: approvals.len(),
        });
    }

    let evidence_commitment = quorum_evidence_commitment(
        proposal_algorithm,
        proposal.project(),
        &expected_proposal,
        &expected_epoch,
        proposal.project_policy(),
        proposal.repository_policy_state(),
        quorum_observed_at_unix_ms,
        threshold,
        &approvals,
    )?;

    Ok(ProjectPolicyReviewQuorumV1 {
        project: proposal.project().clone(),
        proposal: expected_proposal,
        authority_epoch: expected_epoch,
        project_policy: proposal.project_policy().clone(),
        repository_policy_state: proposal.repository_policy_state().clone(),
        quorum_observed_at_unix_ms,
        threshold,
        approvals,
        evidence_commitment,
    })
}

#[allow(clippy::too_many_arguments)]
fn quorum_evidence_commitment(
    algorithm: DigestAlgorithm,
    project: &ProjectIdentity,
    proposal: &ChangeProposalId,
    authority_epoch: &Digest,
    project_policy: &Digest,
    repository_policy_state: &Digest,
    quorum_observed_at_unix_ms: u64,
    threshold: u16,
    approvals: &[QuorumApprovalV1],
) -> Result<Digest, ReviewQuorumError> {
    let count = u32::try_from(approvals.len()).map_err(|_| {
        ReviewQuorumError::CanonicalFieldTooLarge {
            field: "approvals",
            len: approvals.len(),
            max: u32::MAX as usize,
        }
    })?;

    let mut out = Vec::new();
    out.extend_from_slice(REVIEW_QUORUM_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, project)?;
    push_digest(&mut out, proposal.commitment())?;
    push_digest(&mut out, authority_epoch)?;
    push_digest(&mut out, project_policy)?;
    push_digest(&mut out, repository_policy_state)?;
    out.extend_from_slice(&quorum_observed_at_unix_ms.to_be_bytes());
    out.extend_from_slice(&threshold.to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for approval in approvals {
        push_digest(&mut out, approval.reviewer.commitment())?;
        out.extend_from_slice(&approval.review_observed_at_unix_ms.to_be_bytes());
        push_digest(&mut out, &approval.trusted_review_evidence)?;
        push_digest(&mut out, &approval.verifier_identity)?;
    }
    Ok(Digest::of_bytes(algorithm, &out))
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewQuorumError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewQuorumError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewQuorumError> {
    let len = u32::try_from(bytes.len()).map_err(|_| ReviewQuorumError::CanonicalFieldTooLarge {
        field,
        len: bytes.len(),
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Review-quorum evaluation failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewQuorumError {
    /// Proposal and authority epoch belong to different projects, or a review
    /// belongs to another project.
    #[error("review quorum project mismatch")]
    ProjectMismatch,
    /// Proposal/review authority context does not equal the supplied exact
    /// authority epoch.
    #[error("review quorum authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// The supplied common quorum observation time is outside the exact
    /// authority epoch validity window.
    #[error("authority epoch is not valid at the supplied quorum observation time")]
    AuthorityEpochNotValidAtQuorumTime,
    /// Review names another immutable proposal.
    #[error("review quorum mixes change proposals")]
    ProposalMismatch,
    /// Review was trusted under a different project-policy-state commitment.
    #[error("review quorum mixes project-policy states")]
    ProjectPolicyMismatch,
    /// Exact authority epoch has no `ReviewSource` threshold.
    #[error("authority epoch has no ReviewSource threshold")]
    MissingReviewThreshold,
    /// The same reviewer principal appeared more than once.
    #[error("duplicate review-quorum principal: {0}")]
    DuplicateReviewer(PrincipalId),
    /// A purported approval set contains `RequestChanges`.
    #[error("review-quorum input contains non-approval decision by {0}")]
    NonApprovalDecision(PrincipalId),
    /// A review claims an observation after the supplied common quorum time.
    #[error("review by {0} was observed after the supplied quorum time")]
    ReviewObservedAfterQuorumTime(PrincipalId),
    /// A counted reviewer is not eligible for `ReviewSource` at the common
    /// quorum observation time.
    #[error("reviewer is not eligible for ReviewSource at quorum time: {0}")]
    ReviewerNotEligibleAtQuorumTime(PrincipalId),
    /// Distinct approval count is below the exact threshold.
    #[error("review quorum below threshold: {approvals} approvals < {required} required")]
    UnderThreshold {
        /// Required distinct approvals.
        required: u16,
        /// Supplied distinct approvals.
        approvals: usize,
    },
    /// Canonical field exceeded its v1 length bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum length.
        max: usize,
    },
    /// Proposal canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Authority canonicalization failure.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
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
    use mycelix_forge_authority::{AuthorityEpochParts, CapabilityRule, PrincipalGrant};
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review::{
        ReviewStatement, bind_review_evidence, qualify_review_eligibility,
    };
    use mycelix_forge_review_policy::bind_project_policy_trusted_review_v1;
    use mycelix_forge_review_provider::bind_xenia_provider_verified_review_v1;
    use mycelix_forge_xenia::{
        XeniaHybridSuite, XeniaVerificationReceiptV1, xenia_challenge_commitment,
        xenia_key_lineage_commitment, xenia_operator_id_commitment, xenia_provider_namespace,
    };
    use mycelix_forge_xenia_provider::{
        TrustedXeniaVerifierV1, XeniaProviderAttestedReceiptV1,
        provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
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

    fn authority() -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(2_000),
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(principal(0x20), [Capability::ReviewSource]).unwrap(),
                PrincipalGrant::new(principal(0x21), [Capability::ReviewSource]).unwrap(),
                PrincipalGrant::new(principal(0x22), [Capability::ReviewSource]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 2).unwrap(),
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

    struct Context {
        provider: ProviderIdentity,
        authority: AuthorityEpoch,
        project_policy: ProjectPolicyStateV1,
        provider_trust: AuthenticationProviderTrustPolicyV1,
        proposal: ChangeProposal,
    }

    fn context(proposed_revision: u8) -> Context {
        let provider = ProviderIdentity::from_seeds([0x81; 32], [0x82; 32]);
        let trusted = provider.trusted();
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(
                xenia_provider_namespace(),
                trusted.identity_commitment().clone(),
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
        let authority = authority();
        let repository_policy =
            RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        let proposal = ChangeProposal::new(
            principal(0x30),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(proposed_revision),
            git(0x42),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        Context {
            provider,
            authority,
            project_policy,
            provider_trust,
            proposal,
        }
    }

    fn trusted_review(
        context: &Context,
        reviewer: PrincipalId,
        decision: ReviewDecision,
        marker: u8,
        observed_at_unix_ms: u64,
    ) -> ProjectPolicyTrustedReviewV1 {
        let statement = ReviewStatement::new(
            &context.proposal,
            reviewer.clone(),
            decision,
            digest(marker),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let operator = xenia_operator_id_commitment(&format!("operator:{marker:02x}")).unwrap();
        let lineage = xenia_key_lineage_commitment(
            &[marker; 32],
            &[marker.wrapping_add(1); 64],
            None,
        )
        .unwrap();
        let binding = PrincipalBinding::new(
            xenia_provider_namespace(),
            operator.clone(),
            reviewer.clone(),
            lineage.clone(),
        );
        let request = PrincipalAuthenticationRequest::new(
            project(),
            context.authority.digest(DigestAlgorithm::Sha256).unwrap(),
            reviewer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            statement_id.commitment().clone(),
            [marker; 32],
        );
        let receipt = XeniaVerificationReceiptV1::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            operator,
            lineage,
            xenia_challenge_commitment(request.challenge()),
            XeniaHybridSuite::Ed25519MlDsa65V1,
            digest(marker.wrapping_add(2)),
            digest(marker.wrapping_add(3)),
            digest(marker.wrapping_add(4)),
            1_797_000_000,
        );
        let verifier = context.provider.trusted();
        let envelope = context.provider.envelope(receipt);
        let provider_verified = verify_xenia_provider_authentication_v1(
            &verifier,
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
        let bound = bind_review_evidence(
            &context.proposal,
            statement,
            &authentication,
            &context.authority,
        )
        .unwrap();
        let eligible =
            qualify_review_eligibility(bound, &context.authority, observed_at_unix_ms).unwrap();
        let provider_review = bind_xenia_provider_verified_review_v1(
            eligible,
            &authentication,
            &provider_verified,
        )
        .unwrap();
        bind_project_policy_trusted_review_v1(
            &context.proposal,
            &context.project_policy,
            &context.provider_trust,
            provider_review,
        )
        .unwrap()
    }

    #[test]
    fn two_distinct_trusted_approvals_satisfy_two_of_three_threshold() {
        let context = context(0x41);
        let a = trusted_review(&context, principal(0x20), ReviewDecision::Approve, 0x60, 1_200);
        let b = trusted_review(&context, principal(0x21), ReviewDecision::Approve, 0x61, 1_300);
        let quorum = evaluate_project_policy_review_quorum_v1(
            &context.proposal,
            &context.authority,
            1_500,
            &[b, a],
        )
        .unwrap();
        assert_eq!(quorum.threshold(), 2);
        assert_eq!(quorum.approvals().len(), 2);
        assert_eq!(quorum.approvals()[0].reviewer(), &principal(0x20));
        assert_eq!(quorum.approvals()[1].reviewer(), &principal(0x21));
        assert_eq!(quorum.quorum_observed_at_unix_ms(), 1_500);
    }

    #[test]
    fn input_order_does_not_change_quorum_identity() {
        let context = context(0x41);
        let a = trusted_review(&context, principal(0x20), ReviewDecision::Approve, 0x62, 1_200);
        let b = trusted_review(&context, principal(0x21), ReviewDecision::Approve, 0x63, 1_300);
        let left = evaluate_project_policy_review_quorum_v1(
            &context.proposal,
            &context.authority,
            1_500,
            &[a.clone(), b.clone()],
        )
        .unwrap();
        let right = evaluate_project_policy_review_quorum_v1(
            &context.proposal,
            &context.authority,
            1_500,
            &[b, a],
        )
        .unwrap();
        assert_eq!(left.evidence_commitment(), right.evidence_commitment());
        assert_eq!(left.approvals(), right.approvals());
    }

    #[test]
    fn duplicate_reviewer_is_rejected_not_deduplicated() {
        let context = context(0x41);
        let a = trusted_review(&context, principal(0x20), ReviewDecision::Approve, 0x64, 1_200);
        assert_eq!(
            evaluate_project_policy_review_quorum_v1(
                &context.proposal,
                &context.authority,
                1_500,
                &[a.clone(), a],
            )
            .unwrap_err(),
            ReviewQuorumError::DuplicateReviewer(principal(0x20))
        );
    }

    #[test]
    fn request_changes_cannot_count_as_approval() {
        let context = context(0x41);
        let a = trusted_review(&context, principal(0x20), ReviewDecision::Approve, 0x65, 1_200);
        let b = trusted_review(
            &context,
            principal(0x21),
            ReviewDecision::RequestChanges,
            0x66,
            1_300,
        );
        assert_eq!(
            evaluate_project_policy_review_quorum_v1(
                &context.proposal,
                &context.authority,
                1_500,
                &[a, b],
            )
            .unwrap_err(),
            ReviewQuorumError::NonApprovalDecision(principal(0x21))
        );
    }

    #[test]
    fn review_observed_after_common_quorum_time_is_rejected() {
        let context = context(0x41);
        let a = trusted_review(&context, principal(0x20), ReviewDecision::Approve, 0x67, 1_200);
        let b = trusted_review(&context, principal(0x21), ReviewDecision::Approve, 0x68, 1_600);
        assert_eq!(
            evaluate_project_policy_review_quorum_v1(
                &context.proposal,
                &context.authority,
                1_500,
                &[a, b],
            )
            .unwrap_err(),
            ReviewQuorumError::ReviewObservedAfterQuorumTime(principal(0x21))
        );
    }

    #[test]
    fn quorum_time_must_be_inside_exact_authority_epoch() {
        let context = context(0x41);
        let a = trusted_review(&context, principal(0x20), ReviewDecision::Approve, 0x69, 1_200);
        let b = trusted_review(&context, principal(0x21), ReviewDecision::Approve, 0x6a, 1_300);
        assert_eq!(
            evaluate_project_policy_review_quorum_v1(
                &context.proposal,
                &context.authority,
                2_000,
                &[a, b],
            )
            .unwrap_err(),
            ReviewQuorumError::AuthorityEpochNotValidAtQuorumTime
        );
    }

    #[test]
    fn mixed_proposals_are_rejected_even_when_both_reviews_are_individually_trusted() {
        let a_context = context(0x41);
        let b_context = context(0x43);
        let a = trusted_review(
            &a_context,
            principal(0x20),
            ReviewDecision::Approve,
            0x6b,
            1_200,
        );
        let b = trusted_review(
            &b_context,
            principal(0x21),
            ReviewDecision::Approve,
            0x6c,
            1_300,
        );
        assert_eq!(
            evaluate_project_policy_review_quorum_v1(
                &a_context.proposal,
                &a_context.authority,
                1_500,
                &[a, b],
            )
            .unwrap_err(),
            ReviewQuorumError::ProposalMismatch
        );
    }

    #[test]
    fn below_threshold_set_has_no_positive_quorum_type() {
        let context = context(0x41);
        let a = trusted_review(&context, principal(0x20), ReviewDecision::Approve, 0x6d, 1_200);
        assert_eq!(
            evaluate_project_policy_review_quorum_v1(
                &context.proposal,
                &context.authority,
                1_500,
                &[a],
            )
            .unwrap_err(),
            ReviewQuorumError::UnderThreshold {
                required: 2,
                approvals: 1,
            }
        );
    }
}

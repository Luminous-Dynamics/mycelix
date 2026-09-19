// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Distinct-reviewer quorum over exact, structurally authorized Forge reviews.
//!
//! This crate consumes FORGE-007 [`StructurallyAuthorizedReview`] values and
//! proves that one exact change proposal has enough distinct `Approve`
//! decisions to satisfy the exact `ReviewSource` threshold in its bound
//! authority epoch.
//!
//! It does **not** establish trusted time, repository correctness, or merge
//! authority. A later merge-policy theorem must consume this result together
//! with the other required evidence.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review::{ReviewDecision, StructurallyAuthorizedReview};
use serde::Serialize;
use std::collections::BTreeSet;
use thiserror::Error;

const REVIEW_QUORUM_DOMAIN_V1: &[u8] = b"mycelix-forge/review-quorum/v1\0";

/// One normalized approval participating in an established review quorum.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QuorumApproval {
    reviewer: PrincipalId,
    observed_at_unix_ms: u64,
    review_evidence: Digest,
}

impl QuorumApproval {
    /// Distinct reviewer principal counted by this quorum.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// External observation time under which FORGE-007 checked eligibility.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact FORGE-007 structural-review evidence commitment.
    pub fn review_evidence(&self) -> &Digest {
        &self.review_evidence
    }
}

/// Positive result proving that enough distinct exact approvals satisfy the
/// proposal's `ReviewSource` authority threshold.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewQuorum {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    repository_policy_state: Digest,
    threshold: u16,
    approvals: Vec<QuorumApproval>,
    evidence_commitment: Digest,
}

impl ReviewQuorum {
    /// Project containing the proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal subject approved by the quorum.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch under which the quorum was evaluated.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact repository-policy context inherited from the proposal.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Required number of distinct `ReviewSource` principals.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    /// Canonically sorted distinct approval records.
    pub fn approvals(&self) -> &[QuorumApproval] {
        &self.approvals
    }

    /// Aggregate evidence commitment for this exact quorum and reviewer set.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate distinct structurally authorized approvals for one exact proposal.
///
/// Every supplied review is rechecked against the supplied proposal and
/// authority epoch. Duplicate reviewer principals are rejected instead of
/// silently deduplicated. Non-approval decisions are rejected rather than
/// ignored, so a caller cannot accidentally mix `RequestChanges` into an
/// approval set and still receive a positive quorum.
pub fn evaluate_review_quorum(
    proposal: &ChangeProposal,
    authority_epoch: &AuthorityEpoch,
    reviews: &[StructurallyAuthorizedReview],
) -> Result<ReviewQuorum, ReviewQuorumError> {
    if authority_epoch.project() != proposal.project() {
        return Err(ReviewQuorumError::ProjectMismatch);
    }

    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if &expected_epoch != proposal.authority_epoch() {
        return Err(ReviewQuorumError::AuthorityEpochMismatch);
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

    let proposal_algorithm = reviews[0]
        .authenticated_review()
        .statement()
        .proposal()
        .commitment()
        .algorithm();
    let expected_proposal = proposal.proposal_id(proposal_algorithm)?;

    let mut seen = BTreeSet::new();
    let mut approvals = Vec::with_capacity(reviews.len());

    for review in reviews {
        let authenticated = review.authenticated_review();
        let statement = authenticated.statement();

        if authenticated.project() != proposal.project() {
            return Err(ReviewQuorumError::ProjectMismatch);
        }
        if statement.authority_epoch() != proposal.authority_epoch() {
            return Err(ReviewQuorumError::AuthorityEpochMismatch);
        }
        if statement.repository_policy_state() != proposal.repository_policy_state() {
            return Err(ReviewQuorumError::RepositoryPolicyMismatch);
        }
        if statement.proposal() != &expected_proposal {
            return Err(ReviewQuorumError::ProposalMismatch);
        }
        if statement.decision() != ReviewDecision::Approve {
            return Err(ReviewQuorumError::NonApprovalDecision(
                statement.reviewer().clone(),
            ));
        }

        if !authority_epoch.is_principal_eligible(
            statement.reviewer(),
            Capability::ReviewSource,
            review.observed_at_unix_ms(),
        ) {
            return Err(ReviewQuorumError::ReviewerNotEligible(
                statement.reviewer().clone(),
            ));
        }

        if !seen.insert(statement.reviewer().clone()) {
            return Err(ReviewQuorumError::DuplicateReviewer(
                statement.reviewer().clone(),
            ));
        }

        approvals.push(QuorumApproval {
            reviewer: statement.reviewer().clone(),
            observed_at_unix_ms: review.observed_at_unix_ms(),
            review_evidence: review.evidence_commitment().clone(),
        });
    }

    approvals.sort_by(|a, b| a.reviewer.cmp(&b.reviewer));

    if approvals.len() < usize::from(threshold) {
        return Err(ReviewQuorumError::UnderThreshold {
            required: threshold,
            approvals: approvals.len(),
        });
    }

    let evidence_commitment = review_quorum_commitment(
        proposal_algorithm,
        proposal.project(),
        &expected_proposal,
        &expected_epoch,
        proposal.repository_policy_state(),
        threshold,
        &approvals,
    )?;

    Ok(ReviewQuorum {
        project: proposal.project().clone(),
        proposal: expected_proposal,
        authority_epoch: expected_epoch,
        repository_policy_state: proposal.repository_policy_state().clone(),
        threshold,
        approvals,
        evidence_commitment,
    })
}

fn review_quorum_commitment(
    algorithm: DigestAlgorithm,
    project: &ProjectIdentity,
    proposal: &ChangeProposalId,
    authority_epoch: &Digest,
    repository_policy_state: &Digest,
    threshold: u16,
    approvals: &[QuorumApproval],
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
    push_digest(&mut out, repository_policy_state)?;
    out.extend_from_slice(&threshold.to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for approval in approvals {
        push_digest(&mut out, approval.reviewer.commitment())?;
        out.extend_from_slice(&approval.observed_at_unix_ms.to_be_bytes());
        push_digest(&mut out, &approval.review_evidence)?;
    }
    Ok(Digest::of_bytes(algorithm, &out))
}

/// Review-quorum evaluation failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewQuorumError {
    /// Supplied authority epoch belongs to another project.
    #[error("review quorum project does not match authority epoch")]
    ProjectMismatch,
    /// Proposal/review context does not match the supplied authority epoch.
    #[error("review quorum authority epoch does not match")]
    AuthorityEpochMismatch,
    /// Review names another repository-policy state.
    #[error("review quorum mixes repository-policy contexts")]
    RepositoryPolicyMismatch,
    /// Review names another immutable proposal.
    #[error("review quorum mixes change proposals")]
    ProposalMismatch,
    /// Authority epoch has no `ReviewSource` threshold.
    #[error("authority epoch has no ReviewSource threshold")]
    MissingReviewThreshold,
    /// The same principal appeared more than once in the approval set.
    #[error("duplicate review-quorum principal: {0}")]
    DuplicateReviewer(PrincipalId),
    /// A purported approval set contains a security-significant non-approval.
    #[error("review-quorum input contains non-approval decision by {0}")]
    NonApprovalDecision(PrincipalId),
    /// A review no longer satisfies `ReviewSource` structural eligibility.
    #[error("reviewer is not eligible for ReviewSource: {0}")]
    ReviewerNotEligible(PrincipalId),
    /// Distinct approval count is below the exact authority threshold.
    #[error("review quorum below threshold: {approvals} approvals < {required} required")]
    UnderThreshold {
        /// Required distinct approvals.
        required: u16,
        /// Supplied distinct approvals.
        approvals: usize,
    },
    /// Canonical field exceeded its encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Proposal canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Authority canonicalization failure.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
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

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authentication::{
        bind_principal_authentication, AuthenticationObservation, PrincipalAuthenticationRequest,
        PrincipalBinding,
    };
    use mycelix_forge_authority::{
        AuthorityEpochParts, CapabilityRule, PrincipalGrant,
    };
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
    use mycelix_forge_review::{
        bind_authenticated_review, qualify_review_authority, ReviewStatement,
    };

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

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn epoch(project: ProjectIdentity) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project,
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 100,
            valid_until_unix_ms: Some(10_000),
            grants: vec![
                PrincipalGrant::new(
                    principal(0x20),
                    [Capability::ReviewSource, Capability::ManageAuthority],
                )
                .unwrap(),
                PrincipalGrant::new(principal(0x21), [Capability::ReviewSource]).unwrap(),
                PrincipalGrant::new(principal(0x22), [Capability::ReviewSource]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ReviewSource, 2).unwrap(),
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn successor_epoch(previous: &AuthorityEpoch) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project: previous.project().clone(),
            sequence: 1,
            previous: Some(previous.digest(DigestAlgorithm::Sha256).unwrap()),
            valid_from_unix_ms: 100,
            valid_until_unix_ms: Some(10_000),
            grants: previous.grants().to_vec(),
            thresholds: previous.thresholds().to_vec(),
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn proposal(epoch: &AuthorityEpoch, policy: u8, proposed: u8) -> ChangeProposal {
        ChangeProposal::new(
            epoch.project().clone(),
            principal(0x30),
            epoch.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(policy),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(proposed),
            git(0x42),
            digest(0x50),
            vec![],
        )
        .unwrap()
    }

    fn authorized_review(
        proposal: &ChangeProposal,
        epoch: &AuthorityEpoch,
        reviewer: PrincipalId,
        decision: ReviewDecision,
        marker: u8,
        observed_at_unix_ms: u64,
    ) -> StructurallyAuthorizedReview {
        let proposal_id = proposal.proposal_id(DigestAlgorithm::Sha256).unwrap();
        let statement = ReviewStatement::new(
            proposal_id,
            reviewer.clone(),
            proposal.authority_epoch().clone(),
            proposal.repository_policy_state().clone(),
            decision,
            digest(marker),
        );
        let binding = PrincipalBinding::new(
            digest(marker.wrapping_add(1)),
            digest(marker.wrapping_add(2)),
            reviewer.clone(),
            digest(marker.wrapping_add(3)),
        );
        let request = PrincipalAuthenticationRequest::new(
            proposal.project().clone(),
            proposal.authority_epoch().clone(),
            reviewer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            statement.digest(DigestAlgorithm::Sha256).unwrap(),
            [marker; 32],
        );
        let observation = AuthenticationObservation::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(marker.wrapping_add(4)),
            digest(marker.wrapping_add(5)),
        );
        let authentication =
            bind_principal_authentication(request, binding, epoch, observation).unwrap();
        let authenticated =
            bind_authenticated_review(proposal, statement, &authentication).unwrap();
        qualify_review_authority(authenticated, epoch, observed_at_unix_ms).unwrap()
    }

    #[test]
    fn two_of_three_distinct_approvals_establish_quorum() {
        let epoch = epoch(project());
        let proposal = proposal(&epoch, 0x31, 0x41);
        let first = authorized_review(
            &proposal,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        let second = authorized_review(
            &proposal,
            &epoch,
            principal(0x21),
            ReviewDecision::Approve,
            0x70,
            1_001,
        );

        let quorum = evaluate_review_quorum(&proposal, &epoch, &[second, first]).unwrap();
        assert_eq!(quorum.threshold(), 2);
        assert_eq!(quorum.approvals().len(), 2);
        assert_eq!(quorum.approvals()[0].reviewer(), &principal(0x20));
        assert_eq!(quorum.approvals()[1].reviewer(), &principal(0x21));
    }

    #[test]
    fn input_order_does_not_change_quorum_identity() {
        let epoch = epoch(project());
        let proposal = proposal(&epoch, 0x31, 0x41);
        let first = authorized_review(
            &proposal,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        let second = authorized_review(
            &proposal,
            &epoch,
            principal(0x21),
            ReviewDecision::Approve,
            0x70,
            1_001,
        );
        let a = evaluate_review_quorum(&proposal, &epoch, &[first.clone(), second.clone()]).unwrap();
        let b = evaluate_review_quorum(&proposal, &epoch, &[second, first]).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn duplicate_reviewer_is_rejected_not_deduplicated() {
        let epoch = epoch(project());
        let proposal = proposal(&epoch, 0x31, 0x41);
        let review = authorized_review(
            &proposal,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        assert_eq!(
            evaluate_review_quorum(&proposal, &epoch, &[review.clone(), review]).unwrap_err(),
            ReviewQuorumError::DuplicateReviewer(principal(0x20))
        );
    }

    #[test]
    fn one_approval_is_below_two_person_threshold() {
        let epoch = epoch(project());
        let proposal = proposal(&epoch, 0x31, 0x41);
        let review = authorized_review(
            &proposal,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        assert_eq!(
            evaluate_review_quorum(&proposal, &epoch, &[review]).unwrap_err(),
            ReviewQuorumError::UnderThreshold {
                required: 2,
                approvals: 1,
            }
        );
    }

    #[test]
    fn request_changes_cannot_count_as_approval() {
        let epoch = epoch(project());
        let proposal = proposal(&epoch, 0x31, 0x41);
        let approve = authorized_review(
            &proposal,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        let reject = authorized_review(
            &proposal,
            &epoch,
            principal(0x21),
            ReviewDecision::RequestChanges,
            0x70,
            1_001,
        );
        assert_eq!(
            evaluate_review_quorum(&proposal, &epoch, &[approve, reject]).unwrap_err(),
            ReviewQuorumError::NonApprovalDecision(principal(0x21))
        );
    }

    #[test]
    fn proposal_mutation_invalidates_old_quorum_inputs() {
        let epoch = epoch(project());
        let original = proposal(&epoch, 0x31, 0x41);
        let mutated = proposal(&epoch, 0x31, 0x49);
        let first = authorized_review(
            &mutated,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        let second = authorized_review(
            &mutated,
            &epoch,
            principal(0x21),
            ReviewDecision::Approve,
            0x70,
            1_001,
        );
        assert_eq!(
            evaluate_review_quorum(&original, &epoch, &[first, second]).unwrap_err(),
            ReviewQuorumError::ProposalMismatch
        );
    }

    #[test]
    fn mixed_policy_context_is_rejected_before_counting() {
        let epoch = epoch(project());
        let expected = proposal(&epoch, 0x31, 0x41);
        let other = proposal(&epoch, 0x32, 0x41);
        let first = authorized_review(
            &expected,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        let second = authorized_review(
            &other,
            &epoch,
            principal(0x21),
            ReviewDecision::Approve,
            0x70,
            1_001,
        );
        assert_eq!(
            evaluate_review_quorum(&expected, &epoch, &[first, second]).unwrap_err(),
            ReviewQuorumError::RepositoryPolicyMismatch
        );
    }

    #[test]
    fn mixed_authority_epoch_is_rejected_before_counting() {
        let epoch = epoch(project());
        let successor = successor_epoch(&epoch);
        let expected = proposal(&epoch, 0x31, 0x41);
        let other = proposal(&successor, 0x31, 0x41);
        let first = authorized_review(
            &expected,
            &epoch,
            principal(0x20),
            ReviewDecision::Approve,
            0x60,
            1_000,
        );
        let second = authorized_review(
            &other,
            &successor,
            principal(0x21),
            ReviewDecision::Approve,
            0x70,
            1_001,
        );
        assert_eq!(
            evaluate_review_quorum(&expected, &epoch, &[first, second]).unwrap_err(),
            ReviewQuorumError::AuthorityEpochMismatch
        );
    }
}

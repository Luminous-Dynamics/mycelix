// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Distinct structurally authorized approval quorum for Mycelix Forge.
//!
//! This crate proves only that one exact proposal has enough distinct
//! `ReviewSource`-eligible `Approve` reviews under one exact authority epoch at
//! one caller-supplied observation time. It does not prove absence of opposing
//! reviews, trusted time, repository correctness, or merge authorization.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::{
    evaluate_structural_quorum, AuthorityEpoch, Capability, PrincipalId, QuorumError,
};
use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review::{ReviewDecision, StructurallyAuthorizedReview};
use serde::Serialize;
use thiserror::Error;

const APPROVAL_QUORUM_DOMAIN_V1: &[u8] = b"mycelix-forge/approval-quorum/v1\0";

/// Positive evidence that one exact proposal satisfies its structural
/// `ReviewSource` approval threshold.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedApprovalQuorum {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    repository_policy_state: Digest,
    observed_at_unix_ms: u64,
    threshold: u16,
    reviewers: Vec<PrincipalId>,
    review_evidence: Vec<Digest>,
    evidence_commitment: Digest,
}

impl QualifiedApprovalQuorum {
    /// Project whose proposal was reviewed.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal whose approval threshold was met.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch used for quorum evaluation.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact repository-policy state bound by the proposal/reviews.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Caller-supplied observation time used for authority validity.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Required distinct reviewer threshold.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    /// Distinct eligible approving reviewers in canonical order.
    pub fn reviewers(&self) -> &[PrincipalId] {
        &self.reviewers
    }

    /// Review-authority evidence commitments in reviewer order.
    pub fn review_evidence(&self) -> &[Digest] {
        &self.review_evidence
    }

    /// Aggregate approval-quorum commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate a set of positive review-authority objects as one exact approval
/// quorum for `proposal`.
///
/// Every supplied review must be an `Approve` for the exact proposal/context.
/// Duplicate reviewers are rejected by the underlying structural quorum
/// primitive rather than silently deduplicated.
pub fn qualify_approval_quorum(
    proposal: &ChangeProposal,
    authority_epoch: &AuthorityEpoch,
    observed_at_unix_ms: u64,
    reviews: Vec<StructurallyAuthorizedReview>,
) -> Result<QualifiedApprovalQuorum, ApprovalQuorumError> {
    if reviews.is_empty() {
        return Err(ApprovalQuorumError::NoApprovals);
    }
    if authority_epoch.project() != proposal.project() {
        return Err(ApprovalQuorumError::ProjectMismatch);
    }

    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if &expected_epoch != proposal.authority_epoch() {
        return Err(ApprovalQuorumError::AuthorityEpochMismatch);
    }

    let first_algorithm = reviews[0]
        .authenticated_review()
        .statement()
        .proposal()
        .commitment()
        .algorithm();
    let expected_proposal = proposal.proposal_id(first_algorithm)?;

    let mut reviewer_evidence = Vec::with_capacity(reviews.len());
    for review in &reviews {
        let authenticated = review.authenticated_review();
        let statement = authenticated.statement();

        if authenticated.project() != proposal.project() {
            return Err(ApprovalQuorumError::ProjectMismatch);
        }
        if statement.proposal() != &expected_proposal {
            return Err(ApprovalQuorumError::ProposalMismatch);
        }
        if statement.authority_epoch() != proposal.authority_epoch() {
            return Err(ApprovalQuorumError::AuthorityEpochMismatch);
        }
        if statement.repository_policy_state() != proposal.repository_policy_state() {
            return Err(ApprovalQuorumError::RepositoryPolicyMismatch);
        }
        if statement.decision() != ReviewDecision::Approve {
            return Err(ApprovalQuorumError::NonApprovalDecision);
        }

        reviewer_evidence.push((
            statement.reviewer().clone(),
            review.evidence_commitment().clone(),
        ));
    }

    reviewer_evidence.sort_by(|a, b| a.0.cmp(&b.0));
    let reviewers = reviewer_evidence
        .iter()
        .map(|(principal, _)| principal.clone())
        .collect::<Vec<_>>();

    let structural = evaluate_structural_quorum(
        authority_epoch,
        Capability::ReviewSource,
        observed_at_unix_ms,
        &reviewers,
    )?;

    if structural.eligible_authenticated().len() != reviewers.len() {
        return Err(ApprovalQuorumError::ReviewerNoLongerEligible);
    }
    if !structural.satisfied() {
        return Err(ApprovalQuorumError::ThresholdNotSatisfied {
            threshold: structural.threshold(),
            approvals: reviewers.len(),
        });
    }

    let review_evidence = reviewer_evidence
        .iter()
        .map(|(_, evidence)| evidence.clone())
        .collect::<Vec<_>>();
    let algorithm = proposal.authority_epoch().algorithm();
    let evidence_commitment = approval_quorum_commitment(
        algorithm,
        proposal.project(),
        &expected_proposal,
        &expected_epoch,
        proposal.repository_policy_state(),
        observed_at_unix_ms,
        structural.threshold(),
        &reviewer_evidence,
    )?;

    Ok(QualifiedApprovalQuorum {
        project: proposal.project().clone(),
        proposal: expected_proposal,
        authority_epoch: expected_epoch,
        repository_policy_state: proposal.repository_policy_state().clone(),
        observed_at_unix_ms,
        threshold: structural.threshold(),
        reviewers,
        review_evidence,
        evidence_commitment,
    })
}

#[allow(clippy::too_many_arguments)]
fn approval_quorum_commitment(
    algorithm: DigestAlgorithm,
    project: &ProjectIdentity,
    proposal: &ChangeProposalId,
    authority_epoch: &Digest,
    repository_policy_state: &Digest,
    observed_at_unix_ms: u64,
    threshold: u16,
    reviewer_evidence: &[(PrincipalId, Digest)],
) -> Result<Digest, ApprovalQuorumError> {
    let mut out = Vec::new();
    out.extend_from_slice(APPROVAL_QUORUM_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, project)?;
    push_digest(&mut out, proposal.commitment())?;
    push_digest(&mut out, authority_epoch)?;
    push_digest(&mut out, repository_policy_state)?;
    out.extend_from_slice(&observed_at_unix_ms.to_be_bytes());
    out.extend_from_slice(&threshold.to_be_bytes());
    let count = u16::try_from(reviewer_evidence.len()).map_err(|_| {
        ApprovalQuorumError::CanonicalFieldTooLarge {
            field: "reviewers",
            len: reviewer_evidence.len(),
            max: u16::MAX as usize,
        }
    })?;
    out.extend_from_slice(&count.to_be_bytes());
    for (reviewer, evidence) in reviewer_evidence {
        push_digest(&mut out, reviewer.commitment())?;
        push_digest(&mut out, evidence)?;
    }
    Ok(Digest::of_bytes(algorithm, &out))
}

/// Approval-quorum failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ApprovalQuorumError {
    /// No approval evidence was supplied.
    #[error("no approval reviews supplied")]
    NoApprovals,
    /// Authority/project mismatch.
    #[error("approval quorum project mismatch")]
    ProjectMismatch,
    /// Review proposal differs from supplied proposal.
    #[error("approval review names a different proposal")]
    ProposalMismatch,
    /// Authority epoch differs from supplied proposal/epoch.
    #[error("approval quorum authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Repository-policy state differs from proposal.
    #[error("approval review repository-policy state mismatch")]
    RepositoryPolicyMismatch,
    /// A non-Approve review was supplied as approval evidence.
    #[error("non-Approve review cannot count toward approval quorum")]
    NonApprovalDecision,
    /// A previously authorized reviewer is not eligible at quorum observation time.
    #[error("reviewer is not eligible at quorum observation time")]
    ReviewerNoLongerEligible,
    /// Eligible approvals are below the configured threshold.
    #[error("approval threshold not satisfied: {approvals} approvals < {threshold}")]
    ThresholdNotSatisfied {
        /// Required threshold.
        threshold: u16,
        /// Eligible distinct approvals observed.
        approvals: usize,
    },
    /// Canonical field exceeded encoding bounds.
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
    /// Authority/quorum evaluation failed.
    #[error(transparent)]
    Quorum(#[from] QuorumError),
    /// Authority canonicalization failed.
    #[error(transparent)]
    Authority(#[from] mycelix_forge_authority::AuthorityError),
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ApprovalQuorumError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ApprovalQuorumError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ApprovalQuorumError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ApprovalQuorumError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Closed-world join between a witnessed checkpoint review set and the exact
//! policy-adoption-bound trusted reviews named by that checkpoint.
//!
//! FORGE-007G already proves exact authenticated checkpoint membership.
//! FORGE-007K separately proves that one trusted review authenticated the exact
//! project-authorized review-policy context. This crate composes those theorems
//! without re-deriving either.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, ProjectIdentity, ProtocolVersion};
use mycelix_forge_policy_bound_review::PolicyAdoptionBoundTrustedReviewRevisionV1;
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_policy_adoption::ProposalReviewPolicyAuthorityQuorumV1;
use mycelix_forge_review_state::{ReviewRevisionDecision, ReviewRevisionId, ReviewStateError};
use mycelix_forge_witness_checkpoint_review_set::WitnessCheckpointReviewSetV1;
use serde::Serialize;
use std::collections::BTreeMap;
use thiserror::Error;

const POLICY_BOUND_CHECKPOINT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/policy-bound-witness-checkpoint-review-set/v1\0";

/// One exact witnessed checkpoint head whose trusted revision also proved that
/// it authenticated the exact project-authorized review-policy context.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PolicyBoundCheckpointHeadV1 {
    reviewer: mycelix_forge_authority::PrincipalId,
    revision: ReviewRevisionId,
    sequence: u64,
    decision: ReviewRevisionDecision,
    revision_observed_at_unix_ms: u64,
    trusted_revision_evidence: Digest,
    policy_bound_review_evidence: Digest,
    verifier_identity: Digest,
}

impl PolicyBoundCheckpointHeadV1 {
    /// Reviewer owning this exact review lineage head.
    pub fn reviewer(&self) -> &mycelix_forge_authority::PrincipalId {
        &self.reviewer
    }

    /// Exact review revision id named by the witnessed checkpoint.
    pub fn revision(&self) -> &ReviewRevisionId {
        &self.revision
    }

    /// Reviewer-local sequence at the witnessed checkpoint.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Security-significant review state at the checkpoint.
    pub const fn decision(&self) -> ReviewRevisionDecision {
        self.decision
    }

    /// Structural review observation time retained from the trusted review.
    pub const fn revision_observed_at_unix_ms(&self) -> u64 {
        self.revision_observed_at_unix_ms
    }

    /// Exact FORGE-007D project-policy-trusted review evidence.
    pub fn trusted_revision_evidence(&self) -> &Digest {
        &self.trusted_revision_evidence
    }

    /// Exact FORGE-007K policy-adoption-bound review evidence.
    pub fn policy_bound_review_evidence(&self) -> &Digest {
        &self.policy_bound_review_evidence
    }

    /// Exact project-trusted Xenia verifier identity used by the review.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }
}

/// Positive result proving that every head in one witness-qualified checkpoint
/// has exactly one corresponding policy-adoption-bound trusted review, with no
/// missing or extra supplied reviews.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PolicyBoundWitnessCheckpointReviewSetV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    review_policy: Digest,
    review_policy_context: Digest,
    policy_authority_evidence: Digest,
    witness_checkpoint_review_set_evidence: Digest,
    checkpoint_observed_at_unix_ms: u64,
    heads: Vec<PolicyBoundCheckpointHeadV1>,
    evidence_commitment: Digest,
}

impl PolicyBoundWitnessCheckpointReviewSetV1 {
    /// Project containing the exact proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal whose witnessed review heads were policy-bound.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch committed by the proposal/checkpoint.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state committed by the proposal/checkpoint.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state committed by the proposal/checkpoint.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact project-authorized review-policy commitment.
    pub fn review_policy(&self) -> &Digest {
        &self.review_policy
    }

    /// Exact proposal-review policy-context commitment used by every review.
    pub fn review_policy_context(&self) -> &Digest {
        &self.review_policy_context
    }

    /// Exact project-authority evidence adopting the review policy.
    pub fn policy_authority_evidence(&self) -> &Digest {
        &self.policy_authority_evidence
    }

    /// Exact FORGE-007G witnessed checkpoint review-set evidence.
    pub fn witness_checkpoint_review_set_evidence(&self) -> &Digest {
        &self.witness_checkpoint_review_set_evidence
    }

    /// Common structural time at which the Witness threshold accepted the checkpoint.
    pub const fn checkpoint_observed_at_unix_ms(&self) -> u64 {
        self.checkpoint_observed_at_unix_ms
    }

    /// Canonically reviewer-sorted policy-bound checkpoint heads.
    pub fn heads(&self) -> &[PolicyBoundCheckpointHeadV1] {
        &self.heads
    }

    /// Aggregate evidence commitment for this closed-world policy-bound join.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Join one exact witnessed checkpoint review set to the exact set of stronger
/// policy-adoption-bound trusted review revisions.
///
/// The supplied set is closed-world: every checkpoint head must have exactly one
/// policy-bound review and no extra policy-bound review may be supplied.
pub fn bind_policy_bound_witness_checkpoint_review_set_v1(
    proposal: &ChangeProposal,
    checkpoint: &WitnessCheckpointReviewSetV1,
    policy_authority: &ProposalReviewPolicyAuthorityQuorumV1,
    reviews: Vec<PolicyAdoptionBoundTrustedReviewRevisionV1>,
) -> Result<PolicyBoundWitnessCheckpointReviewSetV1, PolicyBoundCheckpointError> {
    let expected_proposal = proposal.proposal_id(checkpoint.proposal().commitment().algorithm())?;

    if checkpoint.project() != proposal.project() || policy_authority.project() != proposal.project() {
        return Err(PolicyBoundCheckpointError::ProjectMismatch);
    }
    if checkpoint.proposal() != &expected_proposal || policy_authority.proposal() != &expected_proposal {
        return Err(PolicyBoundCheckpointError::ProposalMismatch);
    }
    if checkpoint.authority_epoch() != proposal.authority_epoch()
        || policy_authority.authority_epoch() != proposal.authority_epoch()
    {
        return Err(PolicyBoundCheckpointError::AuthorityEpochMismatch);
    }
    if checkpoint.project_policy() != proposal.project_policy()
        || policy_authority.project_policy() != proposal.project_policy()
    {
        return Err(PolicyBoundCheckpointError::ProjectPolicyMismatch);
    }
    if checkpoint.repository_policy_state() != proposal.repository_policy_state() {
        return Err(PolicyBoundCheckpointError::RepositoryPolicyMismatch);
    }

    let mut by_reviewer = BTreeMap::new();
    for review in reviews {
        if review.review_policy() != policy_authority.review_policy()
            || review.review_policy_context() != policy_authority.review_policy_context()
        {
            return Err(PolicyBoundCheckpointError::ReviewPolicyMismatch);
        }
        if review.policy_authority_evidence() != policy_authority.evidence_commitment() {
            return Err(PolicyBoundCheckpointError::PolicyAuthorityEvidenceMismatch);
        }
        let reviewer = review.exact_review_revision().reviewer().clone();
        if by_reviewer.insert(reviewer.clone(), review).is_some() {
            return Err(PolicyBoundCheckpointError::DuplicatePolicyBoundReview(reviewer));
        }
    }

    let mut heads = Vec::with_capacity(checkpoint.heads().len());
    for checkpoint_head in checkpoint.heads() {
        let Some(bound) = by_reviewer.remove(checkpoint_head.reviewer()) else {
            return Err(PolicyBoundCheckpointError::MissingPolicyBoundReview(
                checkpoint_head.reviewer().clone(),
            ));
        };

        let exact = bound.exact_review_revision();
        if exact.sequence() != checkpoint_head.sequence() {
            return Err(PolicyBoundCheckpointError::SequenceMismatch {
                reviewer: checkpoint_head.reviewer().clone(),
                checkpoint: checkpoint_head.sequence(),
                policy_bound: exact.sequence(),
            });
        }
        if exact.decision() != checkpoint_head.decision() {
            return Err(PolicyBoundCheckpointError::DecisionMismatch(
                checkpoint_head.reviewer().clone(),
            ));
        }
        let exact_id = exact.revision_id(checkpoint_head.revision().commitment().algorithm())?;
        if &exact_id != checkpoint_head.revision() {
            return Err(PolicyBoundCheckpointError::RevisionMismatch(
                checkpoint_head.reviewer().clone(),
            ));
        }
        if bound.review_observed_at_unix_ms() != checkpoint_head.revision_observed_at_unix_ms() {
            return Err(PolicyBoundCheckpointError::ObservationTimeMismatch(
                checkpoint_head.reviewer().clone(),
            ));
        }
        if bound.revision().evidence_commitment() != checkpoint_head.trusted_revision_evidence() {
            return Err(PolicyBoundCheckpointError::TrustedReviewEvidenceMismatch(
                checkpoint_head.reviewer().clone(),
            ));
        }
        if bound.revision().verifier_identity() != checkpoint_head.verifier_identity() {
            return Err(PolicyBoundCheckpointError::VerifierIdentityMismatch(
                checkpoint_head.reviewer().clone(),
            ));
        }

        heads.push(PolicyBoundCheckpointHeadV1 {
            reviewer: checkpoint_head.reviewer().clone(),
            revision: checkpoint_head.revision().clone(),
            sequence: checkpoint_head.sequence(),
            decision: checkpoint_head.decision(),
            revision_observed_at_unix_ms: checkpoint_head.revision_observed_at_unix_ms(),
            trusted_revision_evidence: checkpoint_head.trusted_revision_evidence().clone(),
            policy_bound_review_evidence: bound.evidence_commitment().clone(),
            verifier_identity: checkpoint_head.verifier_identity().clone(),
        });
    }

    if let Some((reviewer, _)) = by_reviewer.into_iter().next() {
        return Err(PolicyBoundCheckpointError::ExtraPolicyBoundReview(reviewer));
    }

    let algorithm = checkpoint.evidence_commitment().algorithm();
    let count = u32::try_from(heads.len()).map_err(|_| PolicyBoundCheckpointError::CanonicalFieldTooLarge {
        field: "heads",
        len: heads.len(),
        max: u32::MAX as usize,
    })?;
    let mut out = Vec::new();
    out.extend_from_slice(POLICY_BOUND_CHECKPOINT_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, expected_proposal.commitment())?;
    push_digest(&mut out, proposal.authority_epoch())?;
    push_digest(&mut out, proposal.project_policy())?;
    push_digest(&mut out, proposal.repository_policy_state())?;
    push_digest(&mut out, policy_authority.review_policy())?;
    push_digest(&mut out, policy_authority.review_policy_context())?;
    push_digest(&mut out, policy_authority.evidence_commitment())?;
    push_digest(&mut out, checkpoint.evidence_commitment())?;
    out.extend_from_slice(&checkpoint.checkpoint_observed_at_unix_ms().to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for head in &heads {
        push_digest(&mut out, head.reviewer.commitment())?;
        push_digest(&mut out, head.revision.commitment())?;
        out.extend_from_slice(&head.sequence.to_be_bytes());
        out.push(decision_code(head.decision));
        out.extend_from_slice(&head.revision_observed_at_unix_ms.to_be_bytes());
        push_digest(&mut out, &head.trusted_revision_evidence)?;
        push_digest(&mut out, &head.policy_bound_review_evidence)?;
        push_digest(&mut out, &head.verifier_identity)?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(PolicyBoundWitnessCheckpointReviewSetV1 {
        project: proposal.project().clone(),
        proposal: expected_proposal,
        authority_epoch: proposal.authority_epoch().clone(),
        project_policy: proposal.project_policy().clone(),
        repository_policy_state: proposal.repository_policy_state().clone(),
        review_policy: policy_authority.review_policy().clone(),
        review_policy_context: policy_authority.review_policy_context().clone(),
        policy_authority_evidence: policy_authority.evidence_commitment().clone(),
        witness_checkpoint_review_set_evidence: checkpoint.evidence_commitment().clone(),
        checkpoint_observed_at_unix_ms: checkpoint.checkpoint_observed_at_unix_ms(),
        heads,
        evidence_commitment,
    })
}

fn decision_code(decision: ReviewRevisionDecision) -> u8 {
    match decision {
        ReviewRevisionDecision::Approve => 1,
        ReviewRevisionDecision::RequestChanges => 2,
        ReviewRevisionDecision::Withdraw => 3,
    }
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), PolicyBoundCheckpointError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), PolicyBoundCheckpointError> {
    let len = u32::try_from(bytes.len()).map_err(|_| PolicyBoundCheckpointError::CanonicalFieldTooLarge {
        field,
        len: bytes.len(),
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Policy-bound witnessed-checkpoint join failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum PolicyBoundCheckpointError {
    /// Proposal/checkpoint/policy authority project mismatch.
    #[error("policy-bound checkpoint project mismatch")]
    ProjectMismatch,
    /// Proposal/checkpoint/policy authority proposal mismatch.
    #[error("policy-bound checkpoint proposal mismatch")]
    ProposalMismatch,
    /// Authority epoch differs from the exact proposal context.
    #[error("policy-bound checkpoint authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Project policy differs from the exact proposal context.
    #[error("policy-bound checkpoint project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Repository policy differs from the exact proposal context.
    #[error("policy-bound checkpoint repository-policy mismatch")]
    RepositoryPolicyMismatch,
    /// A supplied policy-bound review names another adopted policy/context.
    #[error("policy-bound checkpoint review-policy mismatch")]
    ReviewPolicyMismatch,
    /// A supplied policy-bound review belongs to another authority-adoption evidence object.
    #[error("policy-bound checkpoint policy-authority evidence mismatch")]
    PolicyAuthorityEvidenceMismatch,
    /// Duplicate policy-bound review for one reviewer.
    #[error("duplicate policy-bound review: {0}")]
    DuplicatePolicyBoundReview(mycelix_forge_authority::PrincipalId),
    /// Checkpoint head has no matching policy-bound review.
    #[error("missing policy-bound review for checkpoint reviewer: {0}")]
    MissingPolicyBoundReview(mycelix_forge_authority::PrincipalId),
    /// Caller supplied a policy-bound review not present in checkpoint.
    #[error("extra policy-bound review not present in checkpoint: {0}")]
    ExtraPolicyBoundReview(mycelix_forge_authority::PrincipalId),
    /// Reviewer-local sequence differs from witnessed checkpoint.
    #[error("policy-bound review sequence mismatch for {reviewer}: checkpoint={checkpoint}, policy-bound={policy_bound}")]
    SequenceMismatch {
        /// Reviewer whose sequence differed.
        reviewer: mycelix_forge_authority::PrincipalId,
        /// Checkpoint sequence.
        checkpoint: u64,
        /// Policy-bound revision sequence.
        policy_bound: u64,
    },
    /// Decision differs from witnessed checkpoint.
    #[error("policy-bound review decision mismatch for {0}")]
    DecisionMismatch(mycelix_forge_authority::PrincipalId),
    /// Revision id differs from witnessed checkpoint.
    #[error("policy-bound review revision mismatch for {0}")]
    RevisionMismatch(mycelix_forge_authority::PrincipalId),
    /// Structural observation time differs from FORGE-007G checkpoint head.
    #[error("policy-bound review observation-time mismatch for {0}")]
    ObservationTimeMismatch(mycelix_forge_authority::PrincipalId),
    /// Underlying trusted-review evidence differs from checkpoint head.
    #[error("policy-bound trusted-review evidence mismatch for {0}")]
    TrustedReviewEvidenceMismatch(mycelix_forge_authority::PrincipalId),
    /// Trusted verifier identity differs from checkpoint head.
    #[error("policy-bound verifier identity mismatch for {0}")]
    VerifierIdentityMismatch(mycelix_forge_authority::PrincipalId),
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
    /// Proposal canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Review revision canonicalization failed.
    #[error(transparent)]
    ReviewState(#[from] ReviewStateError),
}

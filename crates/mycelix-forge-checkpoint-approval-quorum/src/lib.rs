// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Checkpoint-relative approval threshold over exact witness-qualified review
//! heads.
//!
//! FORGE-007G proves exact membership of authenticated trusted review revisions
//! in one witness-qualified checkpoint. This crate asks one narrower question:
//! do the checkpoint heads whose exact state is `Approve` satisfy the exact
//! `Capability::ReviewSource` threshold at that same checkpoint time?
//!
//! A positive [`CheckpointApprovalQuorumV1`] deliberately does not mean the
//! checkpoint is conflict-free or mergeable. `RequestChanges` and `Withdraw`
//! heads remain visible in the result and are never counted as approvals.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{Digest, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_state::{ReviewRevisionDecision, ReviewRevisionId};
use mycelix_forge_witness_checkpoint_review_set::{
    CheckpointReviewHeadV1, WitnessCheckpointReviewSetV1,
};
use serde::Serialize;
use thiserror::Error;

const CHECKPOINT_APPROVAL_QUORUM_DOMAIN_V1: &[u8] =
    b"mycelix-forge/checkpoint-approval-quorum/v1\0";

/// One exact `Approve` head counted by a checkpoint approval quorum.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CountedCheckpointApprovalV1 {
    reviewer: PrincipalId,
    revision: ReviewRevisionId,
    sequence: u64,
    trusted_revision_evidence: Digest,
}

impl CountedCheckpointApprovalV1 {
    /// Reviewer whose exact checkpoint head is `Approve`.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Exact approval revision counted by the quorum.
    pub fn revision(&self) -> &ReviewRevisionId {
        &self.revision
    }

    /// Reviewer-local sequence of the counted approval.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Exact project-policy-trusted review-revision evidence from FORGE-007D/G.
    pub fn trusted_revision_evidence(&self) -> &Digest {
        &self.trusted_revision_evidence
    }
}

/// Positive theorem that the exact checkpoint contains enough distinct eligible
/// `Approve` heads to satisfy the exact `ReviewSource` threshold.
///
/// Presence of principals in [`Self::request_changes_reviewers`] means this
/// theorem is **not** conflict-free. Presence of withdrawn reviewers likewise
/// remains visible. A later explicit review-acceptance policy must interpret
/// those states before merge authorization.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CheckpointApprovalQuorumV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    checkpoint_review_set_evidence: Digest,
    checkpoint_observed_at_unix_ms: u64,
    threshold: u16,
    approvals: Vec<CountedCheckpointApprovalV1>,
    request_changes_reviewers: Vec<PrincipalId>,
    withdrawn_reviewers: Vec<PrincipalId>,
    evidence_commitment: Digest,
}

impl CheckpointApprovalQuorumV1 {
    /// Project containing the exact proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal approved at threshold by this checkpoint.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch used for threshold and eligibility evaluation.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state committed by the proposal.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state committed by the proposal.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact FORGE-007G checkpoint review-set evidence consumed.
    pub fn checkpoint_review_set_evidence(&self) -> &Digest {
        &self.checkpoint_review_set_evidence
    }

    /// Common caller-supplied checkpoint observation time.
    pub const fn checkpoint_observed_at_unix_ms(&self) -> u64 {
        self.checkpoint_observed_at_unix_ms
    }

    /// Exact `ReviewSource` threshold from the authority epoch.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    /// Canonically reviewer-sorted approvals counted by the quorum.
    pub fn approvals(&self) -> &[CountedCheckpointApprovalV1] {
        &self.approvals
    }

    /// Exact checkpoint-head reviewers whose state is `RequestChanges`.
    ///
    /// These principals are not approvals. Their policy effect is deliberately
    /// left to a later review-acceptance theorem.
    pub fn request_changes_reviewers(&self) -> &[PrincipalId] {
        &self.request_changes_reviewers
    }

    /// Exact checkpoint-head reviewers whose state is `Withdraw`.
    pub fn withdrawn_reviewers(&self) -> &[PrincipalId] {
        &self.withdrawn_reviewers
    }

    /// Whether the checkpoint contains any `RequestChanges` head.
    pub fn has_request_changes(&self) -> bool {
        !self.request_changes_reviewers.is_empty()
    }

    /// Aggregate evidence commitment for this exact checkpoint approval theorem.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate the exact `ReviewSource` threshold over `Approve` states in one
/// witness-checkpoint review set.
///
/// Every head is rechecked for `ReviewSource` eligibility at the checkpoint's
/// common observation time. This preserves the authority/time boundary even if
/// the earlier structural eligibility was evaluated at a different time.
pub fn evaluate_checkpoint_approval_quorum_v1(
    proposal: &ChangeProposal,
    authority_epoch: &AuthorityEpoch,
    review_set: &WitnessCheckpointReviewSetV1,
) -> Result<CheckpointApprovalQuorumV1, CheckpointApprovalQuorumError> {
    if review_set.project() != proposal.project() || authority_epoch.project() != proposal.project() {
        return Err(CheckpointApprovalQuorumError::ProjectMismatch);
    }

    let expected_proposal = proposal.proposal_id(review_set.proposal().commitment().algorithm())?;
    if review_set.proposal() != &expected_proposal {
        return Err(CheckpointApprovalQuorumError::ProposalMismatch);
    }

    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch || review_set.authority_epoch() != &expected_epoch {
        return Err(CheckpointApprovalQuorumError::AuthorityEpochMismatch);
    }
    if review_set.project_policy() != proposal.project_policy() {
        return Err(CheckpointApprovalQuorumError::ProjectPolicyMismatch);
    }
    if review_set.repository_policy_state() != proposal.repository_policy_state() {
        return Err(CheckpointApprovalQuorumError::RepositoryPolicyMismatch);
    }

    let checkpoint_time = review_set.checkpoint_observed_at_unix_ms();
    if !authority_epoch.is_valid_at(checkpoint_time) {
        return Err(CheckpointApprovalQuorumError::AuthorityEpochNotValidAtCheckpoint);
    }
    let threshold = authority_epoch
        .threshold_for(Capability::ReviewSource)
        .ok_or(CheckpointApprovalQuorumError::MissingReviewThreshold)?;

    let mut approvals = Vec::new();
    let mut request_changes_reviewers = Vec::new();
    let mut withdrawn_reviewers = Vec::new();

    for head in review_set.heads() {
        if !authority_epoch.is_principal_eligible(
            head.reviewer(),
            Capability::ReviewSource,
            checkpoint_time,
        ) {
            return Err(CheckpointApprovalQuorumError::ReviewerNotEligibleAtCheckpoint(
                head.reviewer().clone(),
            ));
        }

        match head.decision() {
            ReviewRevisionDecision::Approve => approvals.push(counted_approval(head)),
            ReviewRevisionDecision::RequestChanges => {
                request_changes_reviewers.push(head.reviewer().clone());
            }
            ReviewRevisionDecision::Withdraw => {
                withdrawn_reviewers.push(head.reviewer().clone());
            }
        }
    }

    // FORGE-007G heads are canonical by reviewer. Keep this explicit at the
    // threshold boundary so aggregate evidence cannot become order-sensitive
    // if an upstream representation changes later.
    if !is_strictly_sorted_reviewers(approvals.iter().map(|value| &value.reviewer))
        || !is_strictly_sorted_reviewers(request_changes_reviewers.iter())
        || !is_strictly_sorted_reviewers(withdrawn_reviewers.iter())
    {
        return Err(CheckpointApprovalQuorumError::NonCanonicalDecisionSets);
    }

    if approvals.len() < usize::from(threshold) {
        return Err(CheckpointApprovalQuorumError::UnderApprovalThreshold {
            required: threshold,
            actual: approvals.len(),
        });
    }

    let approval_count = u32::try_from(approvals.len()).map_err(|_| {
        CheckpointApprovalQuorumError::CanonicalFieldTooLarge {
            field: "approvals",
            len: approvals.len(),
            max: u32::MAX as usize,
        }
    })?;
    let request_count = u32::try_from(request_changes_reviewers.len()).map_err(|_| {
        CheckpointApprovalQuorumError::CanonicalFieldTooLarge {
            field: "request_changes_reviewers",
            len: request_changes_reviewers.len(),
            max: u32::MAX as usize,
        }
    })?;
    let withdrawn_count = u32::try_from(withdrawn_reviewers.len()).map_err(|_| {
        CheckpointApprovalQuorumError::CanonicalFieldTooLarge {
            field: "withdrawn_reviewers",
            len: withdrawn_reviewers.len(),
            max: u32::MAX as usize,
        }
    })?;

    let algorithm = review_set.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(CHECKPOINT_APPROVAL_QUORUM_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, expected_proposal.commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    push_digest(&mut out, proposal.project_policy())?;
    push_digest(&mut out, proposal.repository_policy_state())?;
    push_digest(&mut out, review_set.evidence_commitment())?;
    out.extend_from_slice(&checkpoint_time.to_be_bytes());
    out.extend_from_slice(&threshold.to_be_bytes());
    out.extend_from_slice(&approval_count.to_be_bytes());
    for approval in &approvals {
        push_digest(&mut out, approval.reviewer.commitment())?;
        push_digest(&mut out, approval.revision.commitment())?;
        out.extend_from_slice(&approval.sequence.to_be_bytes());
        push_digest(&mut out, &approval.trusted_revision_evidence)?;
    }
    out.extend_from_slice(&request_count.to_be_bytes());
    for reviewer in &request_changes_reviewers {
        push_digest(&mut out, reviewer.commitment())?;
    }
    out.extend_from_slice(&withdrawn_count.to_be_bytes());
    for reviewer in &withdrawn_reviewers {
        push_digest(&mut out, reviewer.commitment())?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(CheckpointApprovalQuorumV1 {
        project: proposal.project().clone(),
        proposal: expected_proposal,
        authority_epoch: expected_epoch,
        project_policy: proposal.project_policy().clone(),
        repository_policy_state: proposal.repository_policy_state().clone(),
        checkpoint_review_set_evidence: review_set.evidence_commitment().clone(),
        checkpoint_observed_at_unix_ms: checkpoint_time,
        threshold,
        approvals,
        request_changes_reviewers,
        withdrawn_reviewers,
        evidence_commitment,
    })
}

fn counted_approval(head: &CheckpointReviewHeadV1) -> CountedCheckpointApprovalV1 {
    CountedCheckpointApprovalV1 {
        reviewer: head.reviewer().clone(),
        revision: head.revision().clone(),
        sequence: head.sequence(),
        trusted_revision_evidence: head.trusted_revision_evidence().clone(),
    }
}

fn is_strictly_sorted_reviewers<'a>(mut reviewers: impl Iterator<Item = &'a PrincipalId>) -> bool {
    let Some(mut previous) = reviewers.next() else {
        return true;
    };
    for current in reviewers {
        if previous >= current {
            return false;
        }
        previous = current;
    }
    true
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), CheckpointApprovalQuorumError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), CheckpointApprovalQuorumError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), CheckpointApprovalQuorumError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        CheckpointApprovalQuorumError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Checkpoint approval-quorum evaluation failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum CheckpointApprovalQuorumError {
    /// Project contexts disagree.
    #[error("checkpoint approval quorum project mismatch")]
    ProjectMismatch,
    /// Proposal identities disagree.
    #[error("checkpoint approval quorum proposal mismatch")]
    ProposalMismatch,
    /// Authority epoch commitments disagree.
    #[error("checkpoint approval quorum authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Project-policy commitments disagree.
    #[error("checkpoint approval quorum project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Repository-policy commitments disagree.
    #[error("checkpoint approval quorum repository-policy mismatch")]
    RepositoryPolicyMismatch,
    /// Exact authority epoch is not valid at the checkpoint time.
    #[error("authority epoch is not valid at checkpoint observation time")]
    AuthorityEpochNotValidAtCheckpoint,
    /// Authority epoch has no `ReviewSource` threshold.
    #[error("authority epoch has no ReviewSource threshold")]
    MissingReviewThreshold,
    /// One checkpoint-head reviewer is not eligible at the checkpoint time.
    #[error("reviewer {0} is not eligible for ReviewSource at checkpoint time")]
    ReviewerNotEligibleAtCheckpoint(PrincipalId),
    /// Decision partitions were unexpectedly noncanonical.
    #[error("checkpoint review decision sets are not canonically ordered")]
    NonCanonicalDecisionSets,
    /// Distinct current approvals are below the exact threshold.
    #[error("checkpoint approval quorum below threshold: {actual} < {required}")]
    UnderApprovalThreshold {
        /// Required distinct approvals.
        required: u16,
        /// Distinct checkpoint-head approvals observed.
        actual: usize,
    },
    /// Canonical field exceeded the v1 encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed size.
        len: usize,
        /// Maximum encodable size.
        max: usize,
    },
    /// Authority validation/canonicalization failure.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
    /// Proposal identity/canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{DigestAlgorithm, ForgeCoreError};

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap())
    }

    #[test]
    fn reviewer_order_helper_is_strict() -> Result<(), ForgeCoreError> {
        let a = principal(0x10);
        let b = principal(0x11);
        assert!(is_strictly_sorted_reviewers([&a, &b].into_iter()));
        assert!(!is_strictly_sorted_reviewers([&b, &a].into_iter()));
        assert!(!is_strictly_sorted_reviewers([&a, &a].into_iter()));
        Ok(())
    }
}

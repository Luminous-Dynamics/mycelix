// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evaluate the exact adopted review-acceptance policy over one closed-world,
//! policy-bound witnessed review checkpoint.
//!
//! FORGE-007L establishes that every exact witnessed checkpoint head is the
//! project-policy-trusted authenticated review revision that used the exact
//! project-authorized review-policy context. This crate is therefore allowed to
//! interpret those exact heads under [`ReviewAcceptancePolicyV1`] without
//! trusting an arbitrary caller-selected review subset.
//!
//! ```text
//! PolicyBoundWitnessCheckpointReviewSetV1
//! + exact ReviewAcceptancePolicyV1
//! + exact ProposalReviewPolicyAuthorityQuorumV1
//! + exact AuthorityEpoch
//! + policy-specific RequestChanges semantics
//! + proposer/self-review semantics
//! + required-approver semantics
//! + ReviewSource threshold after policy exclusions
//!     ↓
//! ReviewPolicySatisfiedCheckpointV1
//! ```
//!
//! The positive result remains checkpoint-relative. It does not establish a
//! trusted current clock, that no successor review/checkpoint exists, or merge
//! authorization.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{Digest, ProjectIdentity, ProtocolVersion};
use mycelix_forge_policy_bound_checkpoint::PolicyBoundWitnessCheckpointReviewSetV1;
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_acceptance_policy::{
    ProposerReviewRuleV1, RequestChangesRuleV1, ReviewAcceptancePolicyError,
    ReviewAcceptancePolicyV1,
};
use mycelix_forge_review_policy_adoption::ProposalReviewPolicyAuthorityQuorumV1;
use mycelix_forge_review_state::{ReviewRevisionDecision, ReviewRevisionId};
use serde::Serialize;
use std::collections::BTreeMap;
use thiserror::Error;

const POLICY_SATISFIED_CHECKPOINT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-policy-satisfied-checkpoint/v1\0";

/// One exact checkpoint-head approval counted toward the adopted review policy's
/// `ReviewSource` threshold.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CountedPolicyApprovalV1 {
    reviewer: PrincipalId,
    revision: ReviewRevisionId,
    sequence: u64,
    policy_bound_review_evidence: Digest,
}

impl CountedPolicyApprovalV1 {
    /// Distinct reviewer whose checkpoint head is an admitted counted approval.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Exact review revision counted toward policy satisfaction.
    pub fn revision(&self) -> &ReviewRevisionId {
        &self.revision
    }

    /// Reviewer-local sequence of the counted checkpoint head.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Exact FORGE-007K evidence proving this trusted review used the adopted
    /// policy context.
    pub fn policy_bound_review_evidence(&self) -> &Digest {
        &self.policy_bound_review_evidence
    }
}

/// Positive checkpoint-relative result proving that the exact adopted review
/// policy is satisfied by one exact policy-bound witnessed checkpoint.
///
/// This type intentionally retains non-approval checkpoint state. Under
/// `RequestChangesRuleV1::NonCountingOnly`, a positive result can therefore
/// still report request-changes reviewers.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewPolicySatisfiedCheckpointV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    review_policy: Digest,
    review_policy_context: Digest,
    policy_authority_evidence: Digest,
    policy_bound_checkpoint_evidence: Digest,
    checkpoint_observed_at_unix_ms: u64,
    review_source_threshold: u16,
    counted_approvals: Vec<CountedPolicyApprovalV1>,
    non_counting_approvals: Vec<PrincipalId>,
    request_changes_reviewers: Vec<PrincipalId>,
    withdrawn_reviewers: Vec<PrincipalId>,
    required_approvers: Vec<PrincipalId>,
    evidence_commitment: Digest,
}

impl ReviewPolicySatisfiedCheckpointV1 {
    /// Project containing the exact proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal whose checkpoint satisfied review policy.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch used for reviewer eligibility and threshold.
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

    /// Exact project-authorized review-policy commitment.
    pub fn review_policy(&self) -> &Digest {
        &self.review_policy
    }

    /// Exact proposal-review policy-context commitment used by all checkpoint reviews.
    pub fn review_policy_context(&self) -> &Digest {
        &self.review_policy_context
    }

    /// Exact project-authority quorum evidence adopting the review policy.
    pub fn policy_authority_evidence(&self) -> &Digest {
        &self.policy_authority_evidence
    }

    /// Exact FORGE-007L policy-bound witnessed-checkpoint evidence.
    pub fn policy_bound_checkpoint_evidence(&self) -> &Digest {
        &self.policy_bound_checkpoint_evidence
    }

    /// Common structural checkpoint observation time.
    pub const fn checkpoint_observed_at_unix_ms(&self) -> u64 {
        self.checkpoint_observed_at_unix_ms
    }

    /// Exact `ReviewSource` threshold evaluated under the authority epoch.
    pub const fn review_source_threshold(&self) -> u16 {
        self.review_source_threshold
    }

    /// Canonically reviewer-sorted approvals that counted toward threshold.
    pub fn counted_approvals(&self) -> &[CountedPolicyApprovalV1] {
        &self.counted_approvals
    }

    /// Approvals present at the checkpoint but excluded by policy, currently the
    /// proposer when `MayReviewButDoesNotCount` applies.
    pub fn non_counting_approvals(&self) -> &[PrincipalId] {
        &self.non_counting_approvals
    }

    /// Checkpoint-head reviewers whose exact state is `RequestChanges`.
    pub fn request_changes_reviewers(&self) -> &[PrincipalId] {
        &self.request_changes_reviewers
    }

    /// Whether the policy-satisfied checkpoint still contains request-changes
    /// state. This can be true only when policy uses `NonCountingOnly`.
    pub fn has_request_changes(&self) -> bool {
        !self.request_changes_reviewers.is_empty()
    }

    /// Checkpoint-head reviewers whose exact state is `Withdraw`.
    pub fn withdrawn_reviewers(&self) -> &[PrincipalId] {
        &self.withdrawn_reviewers
    }

    /// Exact canonical required-approver set whose approvals were checked.
    pub fn required_approvers(&self) -> &[PrincipalId] {
        &self.required_approvers
    }

    /// Aggregate review-policy-satisfaction evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PolicyHeadInput {
    reviewer: PrincipalId,
    decision: ReviewRevisionDecision,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PolicyProjection {
    counted_approvers: Vec<PrincipalId>,
    non_counting_approvals: Vec<PrincipalId>,
    request_changes: Vec<PrincipalId>,
    withdrawn: Vec<PrincipalId>,
}

/// Evaluate the adopted review policy over one exact closed-world checkpoint.
///
/// Every checkpoint reviewer is re-checked for `ReviewSource` eligibility at
/// the checkpoint's common structural observation time. The numeric threshold
/// comes exclusively from `AuthorityEpoch`; this policy layer changes only
/// which exact approval heads are allowed to count toward that threshold.
pub fn evaluate_review_policy_satisfied_checkpoint_v1(
    proposal: &ChangeProposal,
    policy: &ReviewAcceptancePolicyV1,
    policy_authority: &ProposalReviewPolicyAuthorityQuorumV1,
    checkpoint: &PolicyBoundWitnessCheckpointReviewSetV1,
    authority_epoch: &AuthorityEpoch,
) -> Result<ReviewPolicySatisfiedCheckpointV1, ReviewPolicySatisfactionError> {
    let expected_proposal = proposal.proposal_id(checkpoint.proposal().commitment().algorithm())?;
    if policy.project() != proposal.project()
        || policy_authority.project() != proposal.project()
        || checkpoint.project() != proposal.project()
        || authority_epoch.project() != proposal.project()
    {
        return Err(ReviewPolicySatisfactionError::ProjectMismatch);
    }
    if policy_authority.proposal() != &expected_proposal || checkpoint.proposal() != &expected_proposal {
        return Err(ReviewPolicySatisfactionError::ProposalMismatch);
    }

    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch
        || policy_authority.authority_epoch() != &expected_epoch
        || checkpoint.authority_epoch() != &expected_epoch
    {
        return Err(ReviewPolicySatisfactionError::AuthorityEpochMismatch);
    }
    if policy_authority.project_policy() != proposal.project_policy()
        || checkpoint.project_policy() != proposal.project_policy()
    {
        return Err(ReviewPolicySatisfactionError::ProjectPolicyMismatch);
    }
    if checkpoint.repository_policy_state() != proposal.repository_policy_state() {
        return Err(ReviewPolicySatisfactionError::RepositoryPolicyMismatch);
    }

    let expected_policy = policy.digest(checkpoint.review_policy().algorithm())?;
    if checkpoint.review_policy() != &expected_policy || policy_authority.review_policy() != &expected_policy {
        return Err(ReviewPolicySatisfactionError::ReviewPolicyMismatch);
    }
    if checkpoint.review_policy_context() != policy_authority.review_policy_context() {
        return Err(ReviewPolicySatisfactionError::ReviewPolicyContextMismatch);
    }
    if checkpoint.policy_authority_evidence() != policy_authority.evidence_commitment() {
        return Err(ReviewPolicySatisfactionError::PolicyAuthorityEvidenceMismatch);
    }

    let checkpoint_time = checkpoint.checkpoint_observed_at_unix_ms();
    if policy_authority.quorum_observed_at_unix_ms() > checkpoint_time {
        return Err(ReviewPolicySatisfactionError::PolicyAdoptedAfterCheckpoint {
            policy_adopted_at_unix_ms: policy_authority.quorum_observed_at_unix_ms(),
            checkpoint_observed_at_unix_ms: checkpoint_time,
        });
    }
    if !authority_epoch.is_valid_at(checkpoint_time) {
        return Err(ReviewPolicySatisfactionError::AuthorityEpochNotValidAtCheckpoint);
    }
    let threshold = authority_epoch
        .threshold_for(Capability::ReviewSource)
        .ok_or(ReviewPolicySatisfactionError::MissingReviewSourceThreshold)?;

    for head in checkpoint.heads() {
        if !authority_epoch.is_principal_eligible(
            head.reviewer(),
            Capability::ReviewSource,
            checkpoint_time,
        ) {
            return Err(ReviewPolicySatisfactionError::ReviewerNotEligibleAtCheckpoint(
                head.reviewer().clone(),
            ));
        }
    }

    let policy_heads: Vec<_> = checkpoint
        .heads()
        .iter()
        .map(|head| PolicyHeadInput {
            reviewer: head.reviewer().clone(),
            decision: head.decision(),
        })
        .collect();
    let projection = evaluate_policy_head_semantics(
        policy,
        proposal.proposer(),
        &policy_heads,
        threshold,
    )?;

    let by_reviewer: BTreeMap<_, _> = checkpoint
        .heads()
        .iter()
        .map(|head| (head.reviewer().clone(), head))
        .collect();
    let mut counted_approvals = Vec::with_capacity(projection.counted_approvers.len());
    for reviewer in &projection.counted_approvers {
        let head = by_reviewer
            .get(reviewer)
            .ok_or_else(|| ReviewPolicySatisfactionError::InternalProjectionMismatch(reviewer.clone()))?;
        counted_approvals.push(CountedPolicyApprovalV1 {
            reviewer: reviewer.clone(),
            revision: head.revision().clone(),
            sequence: head.sequence(),
            policy_bound_review_evidence: head.policy_bound_review_evidence().clone(),
        });
    }

    let algorithm = checkpoint.evidence_commitment().algorithm();
    let count = u32::try_from(counted_approvals.len()).map_err(|_| {
        ReviewPolicySatisfactionError::CanonicalFieldTooLarge {
            field: "counted_approvals",
            len: counted_approvals.len(),
            max: u32::MAX as usize,
        }
    })?;
    let mut out = Vec::new();
    out.extend_from_slice(POLICY_SATISFIED_CHECKPOINT_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, expected_proposal.commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    push_digest(&mut out, proposal.project_policy())?;
    push_digest(&mut out, proposal.repository_policy_state())?;
    push_digest(&mut out, &expected_policy)?;
    push_digest(&mut out, checkpoint.review_policy_context())?;
    push_digest(&mut out, policy_authority.evidence_commitment())?;
    push_digest(&mut out, checkpoint.evidence_commitment())?;
    out.extend_from_slice(&checkpoint_time.to_be_bytes());
    out.extend_from_slice(&threshold.to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for approval in &counted_approvals {
        push_digest(&mut out, approval.reviewer.commitment())?;
        push_digest(&mut out, approval.revision.commitment())?;
        out.extend_from_slice(&approval.sequence.to_be_bytes());
        push_digest(&mut out, &approval.policy_bound_review_evidence)?;
    }
    push_principals(&mut out, "non_counting_approvals", &projection.non_counting_approvals)?;
    push_principals(&mut out, "request_changes", &projection.request_changes)?;
    push_principals(&mut out, "withdrawn", &projection.withdrawn)?;
    push_principals(&mut out, "required_approvers", policy.required_approvers())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ReviewPolicySatisfiedCheckpointV1 {
        project: proposal.project().clone(),
        proposal: expected_proposal,
        authority_epoch: expected_epoch,
        project_policy: proposal.project_policy().clone(),
        repository_policy_state: proposal.repository_policy_state().clone(),
        review_policy: expected_policy,
        review_policy_context: checkpoint.review_policy_context().clone(),
        policy_authority_evidence: policy_authority.evidence_commitment().clone(),
        policy_bound_checkpoint_evidence: checkpoint.evidence_commitment().clone(),
        checkpoint_observed_at_unix_ms: checkpoint_time,
        review_source_threshold: threshold,
        counted_approvals,
        non_counting_approvals: projection.non_counting_approvals,
        request_changes_reviewers: projection.request_changes,
        withdrawn_reviewers: projection.withdrawn,
        required_approvers: policy.required_approvers().to_vec(),
        evidence_commitment,
    })
}

fn evaluate_policy_head_semantics(
    policy: &ReviewAcceptancePolicyV1,
    proposer: &PrincipalId,
    heads: &[PolicyHeadInput],
    threshold: u16,
) -> Result<PolicyProjection, ReviewPolicySatisfactionError> {
    let by_reviewer: BTreeMap<_, _> = heads
        .iter()
        .map(|head| (head.reviewer.clone(), head.decision))
        .collect();
    if by_reviewer.len() != heads.len() {
        return Err(ReviewPolicySatisfactionError::DuplicateReviewerInput);
    }

    if policy.proposer_review_rule() == ProposerReviewRuleV1::Prohibited
        && by_reviewer.contains_key(proposer)
    {
        return Err(ReviewPolicySatisfactionError::ProposerReviewProhibited(
            proposer.clone(),
        ));
    }

    for required in policy.required_approvers() {
        match by_reviewer.get(required) {
            None => {
                return Err(ReviewPolicySatisfactionError::MissingRequiredApprover(
                    required.clone(),
                ));
            }
            Some(ReviewRevisionDecision::Approve) => {}
            Some(decision) => {
                return Err(ReviewPolicySatisfactionError::RequiredApproverNotApproved {
                    principal: required.clone(),
                    decision: *decision,
                });
            }
        }
    }

    let mut counted_approvers = Vec::new();
    let mut non_counting_approvals = Vec::new();
    let mut request_changes = Vec::new();
    let mut withdrawn = Vec::new();

    for head in heads {
        match head.decision {
            ReviewRevisionDecision::Approve => {
                if &head.reviewer == proposer
                    && policy.proposer_review_rule()
                        == ProposerReviewRuleV1::MayReviewButDoesNotCount
                {
                    non_counting_approvals.push(head.reviewer.clone());
                } else {
                    counted_approvers.push(head.reviewer.clone());
                }
            }
            ReviewRevisionDecision::RequestChanges => {
                if policy.request_changes_rule() == RequestChangesRuleV1::BlocksAcceptance {
                    return Err(ReviewPolicySatisfactionError::BlockingRequestChanges(
                        head.reviewer.clone(),
                    ));
                }
                request_changes.push(head.reviewer.clone());
            }
            ReviewRevisionDecision::Withdraw => withdrawn.push(head.reviewer.clone()),
        }
    }

    counted_approvers.sort();
    non_counting_approvals.sort();
    request_changes.sort();
    withdrawn.sort();

    if counted_approvers.len() < usize::from(threshold) {
        return Err(ReviewPolicySatisfactionError::UnderReviewSourceThreshold {
            required: threshold,
            actual: counted_approvers.len(),
        });
    }

    Ok(PolicyProjection {
        counted_approvers,
        non_counting_approvals,
        request_changes,
        withdrawn,
    })
}

fn push_principals(
    out: &mut Vec<u8>,
    field: &'static str,
    principals: &[PrincipalId],
) -> Result<(), ReviewPolicySatisfactionError> {
    let count = u32::try_from(principals.len()).map_err(|_| {
        ReviewPolicySatisfactionError::CanonicalFieldTooLarge {
            field,
            len: principals.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&count.to_be_bytes());
    for principal in principals {
        push_digest(out, principal.commitment())?;
    }
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewPolicySatisfactionError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewPolicySatisfactionError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ReviewPolicySatisfactionError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Review-policy checkpoint evaluation failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReviewPolicySatisfactionError {
    /// Proposal, policy, checkpoint, policy authority or authority epoch project mismatch.
    #[error("review-policy checkpoint project mismatch")]
    ProjectMismatch,
    /// Checkpoint/policy authority names another immutable proposal.
    #[error("review-policy checkpoint proposal mismatch")]
    ProposalMismatch,
    /// Authority epoch differs from the exact proposal/checkpoint context.
    #[error("review-policy checkpoint authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Project policy differs from the exact proposal/checkpoint context.
    #[error("review-policy checkpoint project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Repository policy differs from the exact proposal/checkpoint context.
    #[error("review-policy checkpoint repository-policy mismatch")]
    RepositoryPolicyMismatch,
    /// Supplied review policy differs from the project-authorized checkpoint policy.
    #[error("review-policy checkpoint review-policy mismatch")]
    ReviewPolicyMismatch,
    /// Checkpoint and policy-authority theorem name different review-policy contexts.
    #[error("review-policy checkpoint policy-context mismatch")]
    ReviewPolicyContextMismatch,
    /// Checkpoint does not bind the exact project-authority adoption evidence.
    #[error("review-policy checkpoint policy-authority evidence mismatch")]
    PolicyAuthorityEvidenceMismatch,
    /// Structural policy adoption occurred after the witnessed checkpoint time.
    #[error(
        "review policy adopted after checkpoint: adoption={policy_adopted_at_unix_ms}, checkpoint={checkpoint_observed_at_unix_ms}"
    )]
    PolicyAdoptedAfterCheckpoint {
        /// Caller-supplied common policy-adoption observation time.
        policy_adopted_at_unix_ms: u64,
        /// Caller-supplied witnessed checkpoint observation time.
        checkpoint_observed_at_unix_ms: u64,
    },
    /// Authority epoch was not structurally valid at checkpoint time.
    #[error("authority epoch is not valid at review-policy checkpoint time")]
    AuthorityEpochNotValidAtCheckpoint,
    /// Exact authority epoch has no `ReviewSource` threshold.
    #[error("authority epoch has no ReviewSource threshold")]
    MissingReviewSourceThreshold,
    /// One checkpoint reviewer was not `ReviewSource` eligible at checkpoint time.
    #[error("reviewer {0} is not eligible for ReviewSource at checkpoint time")]
    ReviewerNotEligibleAtCheckpoint(PrincipalId),
    /// `RequestChanges` is a policy blocker under this exact adopted policy.
    #[error("request-changes reviewer blocks acceptance under policy: {0}")]
    BlockingRequestChanges(PrincipalId),
    /// Proposer has a checkpoint review while policy prohibits proposer review.
    #[error("proposer review is prohibited by policy: {0}")]
    ProposerReviewProhibited(PrincipalId),
    /// Required approver has no checkpoint-head review.
    #[error("required approver is missing from checkpoint: {0}")]
    MissingRequiredApprover(PrincipalId),
    /// Required approver exists but its checkpoint head is not `Approve`.
    #[error("required approver {principal} is not approved at checkpoint: {decision:?}")]
    RequiredApproverNotApproved {
        /// Required approver principal.
        principal: PrincipalId,
        /// Exact non-approval checkpoint state.
        decision: ReviewRevisionDecision,
    },
    /// Policy-filtered distinct approval count is below the exact threshold.
    #[error("ReviewSource approval threshold not satisfied: {actual} < {required}")]
    UnderReviewSourceThreshold {
        /// Required distinct counted approvals.
        required: u16,
        /// Actual distinct policy-counted approvals.
        actual: usize,
    },
    /// Internal projection unexpectedly lost a reviewer from the checkpoint map.
    #[error("internal review-policy projection mismatch for reviewer: {0}")]
    InternalProjectionMismatch(PrincipalId),
    /// Duplicate reviewer appeared in the internal policy input.
    #[error("duplicate reviewer in policy evaluation input")]
    DuplicateReviewerInput,
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
    /// Authority validation/canonicalization failed.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
    /// Proposal identity/canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Review-policy validation/canonicalization failed.
    #[error(transparent)]
    ReviewPolicy(#[from] ReviewAcceptancePolicyError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{DigestAlgorithm, ProjectIdentitySeed, GENESIS_NONCE_LEN};

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

    fn policy(
        request_changes_rule: RequestChangesRuleV1,
        proposer_rule: ProposerReviewRuleV1,
        required: Vec<PrincipalId>,
    ) -> ReviewAcceptancePolicyV1 {
        ReviewAcceptancePolicyV1::new(project(), request_changes_rule, proposer_rule, required)
            .unwrap()
    }

    fn head(reviewer: u8, decision: ReviewRevisionDecision) -> PolicyHeadInput {
        PolicyHeadInput {
            reviewer: principal(reviewer),
            decision,
        }
    }

    #[test]
    fn blocking_request_changes_rejects_even_with_enough_approvals() {
        let policy = policy(
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![],
        );
        let heads = vec![
            head(0x20, ReviewRevisionDecision::Approve),
            head(0x21, ReviewRevisionDecision::Approve),
            head(0x22, ReviewRevisionDecision::RequestChanges),
        ];
        assert_eq!(
            evaluate_policy_head_semantics(&policy, &principal(0x30), &heads, 2).unwrap_err(),
            ReviewPolicySatisfactionError::BlockingRequestChanges(principal(0x22))
        );
    }

    #[test]
    fn non_counting_request_changes_can_coexist_with_policy_satisfaction() {
        let policy = policy(
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![],
        );
        let heads = vec![
            head(0x20, ReviewRevisionDecision::Approve),
            head(0x21, ReviewRevisionDecision::Approve),
            head(0x22, ReviewRevisionDecision::RequestChanges),
        ];
        let result = evaluate_policy_head_semantics(&policy, &principal(0x30), &heads, 2).unwrap();
        assert_eq!(result.counted_approvers, vec![principal(0x20), principal(0x21)]);
        assert_eq!(result.request_changes, vec![principal(0x22)]);
    }

    #[test]
    fn proposer_approval_can_be_excluded_from_threshold() {
        let policy = policy(
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![],
        );
        let heads = vec![
            head(0x30, ReviewRevisionDecision::Approve),
            head(0x20, ReviewRevisionDecision::Approve),
        ];
        assert_eq!(
            evaluate_policy_head_semantics(&policy, &principal(0x30), &heads, 2).unwrap_err(),
            ReviewPolicySatisfactionError::UnderReviewSourceThreshold {
                required: 2,
                actual: 1,
            }
        );
    }

    #[test]
    fn prohibited_proposer_review_rejects_any_proposer_head() {
        let policy = policy(
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::Prohibited,
            vec![],
        );
        let heads = vec![head(0x30, ReviewRevisionDecision::Withdraw)];
        assert_eq!(
            evaluate_policy_head_semantics(&policy, &principal(0x30), &heads, 1).unwrap_err(),
            ReviewPolicySatisfactionError::ProposerReviewProhibited(principal(0x30))
        );
    }

    #[test]
    fn missing_required_approver_fails_closed() {
        let policy = policy(
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![principal(0x25)],
        );
        let heads = vec![head(0x20, ReviewRevisionDecision::Approve)];
        assert_eq!(
            evaluate_policy_head_semantics(&policy, &principal(0x30), &heads, 1).unwrap_err(),
            ReviewPolicySatisfactionError::MissingRequiredApprover(principal(0x25))
        );
    }

    #[test]
    fn required_approver_must_be_currently_approved() {
        let policy = policy(
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![principal(0x25)],
        );
        let heads = vec![
            head(0x20, ReviewRevisionDecision::Approve),
            head(0x25, ReviewRevisionDecision::Withdraw),
        ];
        assert_eq!(
            evaluate_policy_head_semantics(&policy, &principal(0x30), &heads, 1).unwrap_err(),
            ReviewPolicySatisfactionError::RequiredApproverNotApproved {
                principal: principal(0x25),
                decision: ReviewRevisionDecision::Withdraw,
            }
        );
    }

    #[test]
    fn proposer_can_count_when_policy_explicitly_allows_it() {
        let policy = policy(
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![],
        );
        let heads = vec![
            head(0x30, ReviewRevisionDecision::Approve),
            head(0x20, ReviewRevisionDecision::Approve),
        ];
        let result = evaluate_policy_head_semantics(&policy, &principal(0x30), &heads, 2).unwrap();
        assert_eq!(result.counted_approvers.len(), 2);
        assert!(result.non_counting_approvals.is_empty());
    }
}

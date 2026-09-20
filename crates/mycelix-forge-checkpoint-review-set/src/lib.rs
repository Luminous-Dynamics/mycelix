// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact authenticated review state projected from one witness-qualified Forge
//! checkpoint.
//!
//! A witness-qualified review-head snapshot proves that project-authorized
//! Witness principals accepted one exact completeness checkpoint. It still does
//! not prove that the opaque head ids correspond to the caller's supplied
//! authenticated review-revision objects. This crate closes that substitution
//! boundary with an exact bijection.
//!
//! ```text
//! WitnessQualifiedReviewHeadSnapshotV1
//! + exact EvidenceBoundReviewHeadSnapshotV1
//! + exact ProjectPolicyTrustedReviewRevisionV1 set
//! + reviewer/revision-id/sequence bijection
//!     ↓
//! WitnessQualifiedCheckpointReviewSetV1
//! ```
//!
//! The result is checkpoint-relative. It is deliberately not named
//! `CurrentReviewSet`: later review revisions may exist after the witnessed
//! checkpoint, and the checkpoint's time is not a trusted global clock.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_head_snapshot::{
    EvidenceBoundReviewHeadSnapshotV1, ReviewHeadSnapshotError,
};
use mycelix_forge_review_head_witness::WitnessQualifiedReviewHeadSnapshotV1;
use mycelix_forge_review_state::{ReviewRevisionDecision, ReviewStateError};
use mycelix_forge_review_state_auth::ProjectPolicyTrustedReviewRevisionV1;
use serde::Serialize;
use std::collections::BTreeMap;
use thiserror::Error;

const CHECKPOINT_REVIEW_SET_DOMAIN_V1: &[u8] =
    b"mycelix-forge/witness-qualified-checkpoint-review-set/v1\0";

/// One exact authenticated reviewer state present in a witness-qualified
/// checkpoint.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CheckpointReviewStateV1 {
    reviewer: mycelix_forge_authority::PrincipalId,
    revision: mycelix_forge_review_state::ReviewRevisionId,
    sequence: u64,
    decision: ReviewRevisionDecision,
    review_context: Digest,
    original_eligibility_observed_at_unix_ms: u64,
    trusted_revision_evidence: Digest,
}

impl CheckpointReviewStateV1 {
    /// Reviewer whose exact state this entry represents.
    pub fn reviewer(&self) -> &mycelix_forge_authority::PrincipalId {
        &self.reviewer
    }

    /// Exact authenticated revision id named by the checkpoint.
    pub fn revision(&self) -> &mycelix_forge_review_state::ReviewRevisionId {
        &self.revision
    }

    /// Reviewer-local revision sequence.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Exact authenticated decision at this checkpoint.
    pub const fn decision(&self) -> ReviewRevisionDecision {
        self.decision
    }

    /// Immutable review context committed by the exact revision.
    pub fn review_context(&self) -> &Digest {
        &self.review_context
    }

    /// Caller-supplied time at which the revision previously passed structural
    /// `ReviewSource` eligibility.
    pub const fn original_eligibility_observed_at_unix_ms(&self) -> u64 {
        self.original_eligibility_observed_at_unix_ms
    }

    /// Exact project-policy-trusted review-revision evidence commitment.
    pub fn trusted_revision_evidence(&self) -> &Digest {
        &self.trusted_revision_evidence
    }
}

/// Positive result proving an exact one-to-one correspondence between all heads
/// in one witness-qualified snapshot and all supplied project-policy-trusted
/// review revisions.
///
/// This type is serializable for evidence export but intentionally not
/// deserializable into positive authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct WitnessQualifiedCheckpointReviewSetV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    snapshot_evidence: Digest,
    witness_checkpoint_evidence: Digest,
    checkpoint_observed_at_unix_ms: u64,
    reviews: Vec<CheckpointReviewStateV1>,
    evidence_commitment: Digest,
}

impl WitnessQualifiedCheckpointReviewSetV1 {
    /// Project containing the exact proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal whose review state was projected.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch inherited from the proposal/checkpoint.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state inherited from the proposal/checkpoint.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state inherited from the proposal/checkpoint.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact evidence-bound review-head snapshot used by this projection.
    pub fn snapshot_evidence(&self) -> &Digest {
        &self.snapshot_evidence
    }

    /// Exact witness-qualified checkpoint evidence commitment.
    pub fn witness_checkpoint_evidence(&self) -> &Digest {
        &self.witness_checkpoint_evidence
    }

    /// Caller-supplied common witness quorum observation time inherited from the
    /// checkpoint.
    pub const fn checkpoint_observed_at_unix_ms(&self) -> u64 {
        self.checkpoint_observed_at_unix_ms
    }

    /// Canonically reviewer-ordered exact authenticated states at the checkpoint.
    pub fn reviews(&self) -> &[CheckpointReviewStateV1] {
        &self.reviews
    }

    /// Aggregate checkpoint review-set evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Project exact authenticated review state from one witness-qualified review
/// head checkpoint.
///
/// The supplied trusted revisions must form an exact bijection with the
/// snapshot's reviewer heads: no missing reviewer, no extra reviewer, no
/// duplicate reviewer, and no revision-id/sequence substitution.
pub fn project_witness_qualified_checkpoint_review_set_v1(
    proposal: &ChangeProposal,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
    checkpoint: &WitnessQualifiedReviewHeadSnapshotV1,
    trusted_revisions: &[ProjectPolicyTrustedReviewRevisionV1],
) -> Result<WitnessQualifiedCheckpointReviewSetV1, CheckpointReviewSetError> {
    validate_checkpoint_context(proposal, snapshot, checkpoint)?;

    let heads = snapshot.snapshot().heads();
    if heads.len() != trusted_revisions.len() {
        return Err(CheckpointReviewSetError::RevisionCountMismatch {
            heads: heads.len(),
            revisions: trusted_revisions.len(),
        });
    }

    let mut revisions_by_reviewer = BTreeMap::new();
    for trusted in trusted_revisions {
        let exact = exact_revision(trusted);
        if revisions_by_reviewer
            .insert(exact.reviewer().clone(), trusted)
            .is_some()
        {
            return Err(CheckpointReviewSetError::DuplicateTrustedReviewer(
                exact.reviewer().clone(),
            ));
        }
    }

    let mut reviews = Vec::with_capacity(heads.len());
    for head in heads {
        let trusted = revisions_by_reviewer
            .get(head.reviewer())
            .copied()
            .ok_or_else(|| {
                CheckpointReviewSetError::MissingTrustedRevision(head.reviewer().clone())
            })?;
        let exact = exact_revision(trusted);

        if exact.proposal() != snapshot.snapshot().proposal() {
            return Err(CheckpointReviewSetError::RevisionProposalMismatch(
                head.reviewer().clone(),
            ));
        }
        if trusted.project_policy() != proposal.project_policy() {
            return Err(CheckpointReviewSetError::RevisionProjectPolicyMismatch(
                head.reviewer().clone(),
            ));
        }
        if exact.sequence() != head.sequence() {
            return Err(CheckpointReviewSetError::RevisionSequenceMismatch {
                reviewer: head.reviewer().clone(),
                expected: head.sequence(),
                actual: exact.sequence(),
            });
        }

        let expected_revision = exact.revision_id(head.revision().commitment().algorithm())?;
        if &expected_revision != head.revision() {
            return Err(CheckpointReviewSetError::RevisionIdentityMismatch(
                head.reviewer().clone(),
            ));
        }

        let observed_at = trusted_observed_at_unix_ms(trusted);
        if observed_at > checkpoint.quorum_observed_at_unix_ms() {
            return Err(CheckpointReviewSetError::RevisionObservedAfterCheckpoint {
                reviewer: head.reviewer().clone(),
                revision_observed_at: observed_at,
                checkpoint_observed_at: checkpoint.quorum_observed_at_unix_ms(),
            });
        }

        reviews.push(CheckpointReviewStateV1 {
            reviewer: head.reviewer().clone(),
            revision: expected_revision,
            sequence: exact.sequence(),
            decision: exact.decision(),
            review_context: exact.review_context().clone(),
            original_eligibility_observed_at_unix_ms: observed_at,
            trusted_revision_evidence: trusted.evidence_commitment().clone(),
        });
    }

    let algorithm = checkpoint.evidence_commitment().algorithm();
    let proposal_id = proposal.proposal_id(snapshot.snapshot().proposal().commitment().algorithm())?;
    let count = u32::try_from(reviews.len()).map_err(|_| {
        CheckpointReviewSetError::CanonicalFieldTooLarge {
            field: "reviews",
            len: reviews.len(),
            max: u32::MAX as usize,
        }
    })?;
    let mut out = Vec::new();
    out.extend_from_slice(CHECKPOINT_REVIEW_SET_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, proposal_id.commitment())?;
    push_digest(&mut out, proposal.authority_epoch())?;
    push_digest(&mut out, proposal.project_policy())?;
    push_digest(&mut out, proposal.repository_policy_state())?;
    push_digest(&mut out, snapshot.evidence_commitment())?;
    push_digest(&mut out, checkpoint.evidence_commitment())?;
    out.extend_from_slice(&checkpoint.quorum_observed_at_unix_ms().to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for review in &reviews {
        push_digest(&mut out, review.reviewer.commitment())?;
        push_digest(&mut out, review.revision.commitment())?;
        out.extend_from_slice(&review.sequence.to_be_bytes());
        out.push(review_decision_code(review.decision));
        push_digest(&mut out, &review.review_context)?;
        out.extend_from_slice(
            &review
                .original_eligibility_observed_at_unix_ms
                .to_be_bytes(),
        );
        push_digest(&mut out, &review.trusted_revision_evidence)?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(WitnessQualifiedCheckpointReviewSetV1 {
        project: proposal.project().clone(),
        proposal: proposal_id,
        authority_epoch: proposal.authority_epoch().clone(),
        project_policy: proposal.project_policy().clone(),
        repository_policy_state: proposal.repository_policy_state().clone(),
        snapshot_evidence: snapshot.evidence_commitment().clone(),
        witness_checkpoint_evidence: checkpoint.evidence_commitment().clone(),
        checkpoint_observed_at_unix_ms: checkpoint.quorum_observed_at_unix_ms(),
        reviews,
        evidence_commitment,
    })
}

fn validate_checkpoint_context(
    proposal: &ChangeProposal,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
    checkpoint: &WitnessQualifiedReviewHeadSnapshotV1,
) -> Result<(), CheckpointReviewSetError> {
    let claim = snapshot.snapshot();
    let expected_proposal = proposal.proposal_id(claim.proposal().commitment().algorithm())?;

    if claim.project() != proposal.project() || checkpoint.project() != proposal.project() {
        return Err(CheckpointReviewSetError::ProjectMismatch);
    }
    if claim.proposal() != &expected_proposal || checkpoint.proposal() != &expected_proposal {
        return Err(CheckpointReviewSetError::ProposalMismatch);
    }
    if claim.authority_epoch() != proposal.authority_epoch()
        || checkpoint.authority_epoch() != proposal.authority_epoch()
    {
        return Err(CheckpointReviewSetError::AuthorityEpochMismatch);
    }
    if claim.project_policy() != proposal.project_policy()
        || checkpoint.project_policy() != proposal.project_policy()
    {
        return Err(CheckpointReviewSetError::ProjectPolicyMismatch);
    }
    if claim.repository_policy_state() != proposal.repository_policy_state()
        || checkpoint.repository_policy_state() != proposal.repository_policy_state()
    {
        return Err(CheckpointReviewSetError::RepositoryPolicyMismatch);
    }
    if checkpoint.snapshot_evidence() != snapshot.evidence_commitment() {
        return Err(CheckpointReviewSetError::CheckpointSnapshotMismatch);
    }
    if checkpoint.provider_checkpoint() != claim.provider_checkpoint() {
        return Err(CheckpointReviewSetError::ProviderCheckpointMismatch);
    }
    if checkpoint.provider_namespace() != snapshot.observation().provider_namespace() {
        return Err(CheckpointReviewSetError::ProviderNamespaceMismatch);
    }
    Ok(())
}

fn exact_revision(
    trusted: &ProjectPolicyTrustedReviewRevisionV1,
) -> &mycelix_forge_review_state::ReviewRevisionV1 {
    trusted.revision().revision().revision().revision()
}

fn trusted_observed_at_unix_ms(trusted: &ProjectPolicyTrustedReviewRevisionV1) -> u64 {
    trusted.revision().revision().observed_at_unix_ms()
}

const fn review_decision_code(decision: ReviewRevisionDecision) -> u8 {
    match decision {
        ReviewRevisionDecision::Approve => 1,
        ReviewRevisionDecision::RequestChanges => 2,
        ReviewRevisionDecision::Withdraw => 3,
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), CheckpointReviewSetError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), CheckpointReviewSetError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), CheckpointReviewSetError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        CheckpointReviewSetError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Checkpoint review-set projection failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum CheckpointReviewSetError {
    /// Snapshot/checkpoint and proposal belong to different projects.
    #[error("checkpoint review-set project mismatch")]
    ProjectMismatch,
    /// Snapshot/checkpoint names another proposal.
    #[error("checkpoint review-set proposal mismatch")]
    ProposalMismatch,
    /// Snapshot/checkpoint authority epoch differs from the proposal.
    #[error("checkpoint review-set authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Snapshot/checkpoint project policy differs from the proposal.
    #[error("checkpoint review-set project policy mismatch")]
    ProjectPolicyMismatch,
    /// Snapshot/checkpoint repository policy differs from the proposal.
    #[error("checkpoint review-set repository policy mismatch")]
    RepositoryPolicyMismatch,
    /// Witness checkpoint was produced for a different evidence-bound snapshot.
    #[error("witness checkpoint does not qualify the supplied snapshot")]
    CheckpointSnapshotMismatch,
    /// Provider checkpoint commitments disagree.
    #[error("witness checkpoint provider checkpoint mismatch")]
    ProviderCheckpointMismatch,
    /// Provider namespace differs between snapshot and witness checkpoint.
    #[error("witness checkpoint provider namespace mismatch")]
    ProviderNamespaceMismatch,
    /// Number of trusted revision objects differs from number of snapshot heads.
    #[error("checkpoint head/revision count mismatch: {heads} heads != {revisions} revisions")]
    RevisionCountMismatch {
        /// Snapshot head count.
        heads: usize,
        /// Supplied trusted revision count.
        revisions: usize,
    },
    /// Supplied trusted revision list contains the same reviewer more than once.
    #[error("duplicate trusted review revision for reviewer {0}")]
    DuplicateTrustedReviewer(mycelix_forge_authority::PrincipalId),
    /// Snapshot names a reviewer with no supplied project-policy-trusted revision.
    #[error("missing trusted review revision for snapshot reviewer {0}")]
    MissingTrustedRevision(mycelix_forge_authority::PrincipalId),
    /// Trusted revision names another proposal.
    #[error("trusted review revision for {0} names another proposal")]
    RevisionProposalMismatch(mycelix_forge_authority::PrincipalId),
    /// Trusted revision was qualified under another project-policy state.
    #[error("trusted review revision for {0} uses another project policy")]
    RevisionProjectPolicyMismatch(mycelix_forge_authority::PrincipalId),
    /// Reviewer-local sequence differs from the snapshot claim.
    #[error("trusted review revision sequence mismatch for {reviewer}: expected {expected}, got {actual}")]
    RevisionSequenceMismatch {
        /// Reviewer whose sequence differed.
        reviewer: mycelix_forge_authority::PrincipalId,
        /// Sequence named by snapshot head.
        expected: u64,
        /// Sequence embedded in trusted revision.
        actual: u64,
    },
    /// Exact trusted revision id differs from the snapshot head id.
    #[error("trusted review revision identity mismatch for {0}")]
    RevisionIdentityMismatch(mycelix_forge_authority::PrincipalId),
    /// Revision's structural eligibility observation is after the witness
    /// checkpoint's common observation time.
    #[error("trusted revision for {reviewer} was observed at {revision_observed_at}, after checkpoint time {checkpoint_observed_at}")]
    RevisionObservedAfterCheckpoint {
        /// Reviewer whose revision was observed too late.
        reviewer: mycelix_forge_authority::PrincipalId,
        /// Original structural eligibility observation time.
        revision_observed_at: u64,
        /// Witness checkpoint common observation time.
        checkpoint_observed_at: u64,
    },
    /// Canonical field exceeded the v1 length bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Proposal identity/canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Review revision identity/canonicalization failure.
    #[error(transparent)]
    ReviewState(#[from] ReviewStateError),
    /// Snapshot canonicalization failure.
    #[error(transparent)]
    Snapshot(#[from] ReviewHeadSnapshotError),
}

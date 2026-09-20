// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact membership join between a witness-qualified review-head checkpoint and
//! the project-policy-trusted authenticated review revisions named by it.
//!
//! FORGE-007F establishes that the project's exact `Witness` threshold accepted
//! one exact provider completeness checkpoint. That theorem does not by itself
//! prove that the portable head identifiers in the snapshot are the same
//! authenticated review revisions a later consumer supplies. This crate closes
//! that substitution gap.
//!
//! ```text
//! WitnessQualifiedReviewHeadSnapshotV1
//! + exact EvidenceBoundReviewHeadSnapshotV1
//! + exact ProjectPolicyTrustedReviewRevisionV1 set
//! + 1:1 reviewer / revision-id / sequence equality
//!     ↓
//! WitnessCheckpointReviewSetV1
//! ```
//!
//! The result is deliberately checkpoint-relative. It does **not** prove that
//! no successor review exists after the checkpoint, that the checkpoint remains
//! fresh now, that approvals satisfy a review threshold, or that merge is
//! authorized.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::PrincipalId;
use mycelix_forge_core::{Digest, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_head_snapshot::EvidenceBoundReviewHeadSnapshotV1;
use mycelix_forge_review_head_witness::WitnessQualifiedReviewHeadSnapshotV1;
use mycelix_forge_review_state::{ReviewRevisionDecision, ReviewRevisionId, ReviewStateError};
use mycelix_forge_review_state_auth::ProjectPolicyTrustedReviewRevisionV1;
use serde::Serialize;
use std::collections::BTreeMap;
use thiserror::Error;

const CHECKPOINT_REVIEW_SET_DOMAIN_V1: &[u8] =
    b"mycelix-forge/witness-checkpoint-review-set/v1\0";

/// One exact authenticated review head admitted by the witness-qualified
/// checkpoint membership join.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CheckpointReviewHeadV1 {
    reviewer: PrincipalId,
    revision: ReviewRevisionId,
    sequence: u64,
    decision: ReviewRevisionDecision,
    revision_observed_at_unix_ms: u64,
    trusted_revision_evidence: Digest,
    verifier_identity: Digest,
}

impl CheckpointReviewHeadV1 {
    /// Reviewer owning this exact lineage head.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Exact authenticated revision id named by the checkpoint.
    pub fn revision(&self) -> &ReviewRevisionId {
        &self.revision
    }

    /// Exact reviewer-local sequence named by the checkpoint.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Security-significant review state at this checkpoint head.
    pub const fn decision(&self) -> ReviewRevisionDecision {
        self.decision
    }

    /// Caller-supplied structural observation time carried by FORGE-007D.
    ///
    /// This remains an untrusted-time field. Its inclusion prevents a revision
    /// explicitly observed after the checkpoint quorum time from being admitted
    /// retroactively, but does not create a trusted clock theorem.
    pub const fn revision_observed_at_unix_ms(&self) -> u64 {
        self.revision_observed_at_unix_ms
    }

    /// Exact FORGE-007D project-policy-trusted revision evidence commitment.
    pub fn trusted_revision_evidence(&self) -> &Digest {
        &self.trusted_revision_evidence
    }

    /// Exact project-trusted Xenia verifier identity for this review revision.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }
}

/// Positive checkpoint-relative projection of the exact trusted review heads
/// named by one witness-qualified snapshot.
///
/// This type is serializable for evidence export but intentionally not
/// deserializable into authority. Construction requires re-running
/// [`bind_witness_checkpoint_review_set_v1`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct WitnessCheckpointReviewSetV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    snapshot_evidence: Digest,
    witness_checkpoint_evidence: Digest,
    provider_namespace: Digest,
    provider_checkpoint: Digest,
    checkpoint_observed_at_unix_ms: u64,
    heads: Vec<CheckpointReviewHeadV1>,
    evidence_commitment: Digest,
}

impl WitnessCheckpointReviewSetV1 {
    /// Project containing the exact proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal whose checkpoint review set was joined.
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

    /// Exact FORGE-007E evidence-bound snapshot commitment.
    pub fn snapshot_evidence(&self) -> &Digest {
        &self.snapshot_evidence
    }

    /// Exact FORGE-007F witness-qualified checkpoint evidence commitment.
    pub fn witness_checkpoint_evidence(&self) -> &Digest {
        &self.witness_checkpoint_evidence
    }

    /// Collaboration-state provider namespace carried by the checkpoint.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Opaque concrete-provider checkpoint commitment.
    pub fn provider_checkpoint(&self) -> &Digest {
        &self.provider_checkpoint
    }

    /// Common caller-supplied time at which the Witness threshold accepted the
    /// checkpoint.
    pub const fn checkpoint_observed_at_unix_ms(&self) -> u64 {
        self.checkpoint_observed_at_unix_ms
    }

    /// Canonically reviewer-sorted exact authenticated heads.
    pub fn heads(&self) -> &[CheckpointReviewHeadV1] {
        &self.heads
    }

    /// Aggregate evidence commitment for this exact membership join.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Join one witness-qualified checkpoint to the exact project-policy-trusted
/// authenticated review revisions named by its underlying snapshot.
///
/// The join is closed-world with respect to the supplied snapshot: every
/// snapshot head must have exactly one matching trusted revision and no extra
/// trusted revision may be supplied. A matching reviewer with a different
/// revision id or sequence fails rather than being normalized.
pub fn bind_witness_checkpoint_review_set_v1(
    proposal: &ChangeProposal,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
    witnessed: &WitnessQualifiedReviewHeadSnapshotV1,
    revisions: Vec<ProjectPolicyTrustedReviewRevisionV1>,
) -> Result<WitnessCheckpointReviewSetV1, WitnessCheckpointReviewSetError> {
    let claim = snapshot.snapshot();
    let expected_proposal = proposal.proposal_id(claim.proposal().commitment().algorithm())?;

    if claim.project() != proposal.project() || witnessed.project() != proposal.project() {
        return Err(WitnessCheckpointReviewSetError::ProjectMismatch);
    }
    if claim.proposal() != &expected_proposal || witnessed.proposal() != &expected_proposal {
        return Err(WitnessCheckpointReviewSetError::ProposalMismatch);
    }
    if claim.authority_epoch() != proposal.authority_epoch()
        || witnessed.authority_epoch() != proposal.authority_epoch()
    {
        return Err(WitnessCheckpointReviewSetError::AuthorityEpochMismatch);
    }
    if claim.project_policy() != proposal.project_policy()
        || witnessed.project_policy() != proposal.project_policy()
    {
        return Err(WitnessCheckpointReviewSetError::ProjectPolicyMismatch);
    }
    if claim.repository_policy_state() != proposal.repository_policy_state()
        || witnessed.repository_policy_state() != proposal.repository_policy_state()
    {
        return Err(WitnessCheckpointReviewSetError::RepositoryPolicyMismatch);
    }
    if witnessed.snapshot_evidence() != snapshot.evidence_commitment() {
        return Err(WitnessCheckpointReviewSetError::SnapshotEvidenceMismatch);
    }
    if witnessed.provider_namespace() != snapshot.observation().provider_namespace() {
        return Err(WitnessCheckpointReviewSetError::ProviderNamespaceMismatch);
    }
    if witnessed.provider_checkpoint() != claim.provider_checkpoint() {
        return Err(WitnessCheckpointReviewSetError::ProviderCheckpointMismatch);
    }

    let mut by_reviewer = BTreeMap::new();
    for trusted in revisions {
        let reviewer = trusted_exact_revision(&trusted).reviewer().clone();
        if by_reviewer.insert(reviewer.clone(), trusted).is_some() {
            return Err(WitnessCheckpointReviewSetError::DuplicateTrustedRevision(
                reviewer,
            ));
        }
    }

    let mut heads = Vec::with_capacity(claim.heads().len());
    for claimed in claim.heads() {
        let Some(trusted) = by_reviewer.remove(claimed.reviewer()) else {
            return Err(WitnessCheckpointReviewSetError::MissingTrustedRevision(
                claimed.reviewer().clone(),
            ));
        };

        let provider_verified = trusted.revision();
        let eligible = provider_verified.revision();
        let evidence_bound = eligible.revision();
        let exact = evidence_bound.revision();

        if evidence_bound.project() != proposal.project() {
            return Err(WitnessCheckpointReviewSetError::ProjectMismatch);
        }
        if evidence_bound.proposal() != &expected_proposal || exact.proposal() != &expected_proposal {
            return Err(WitnessCheckpointReviewSetError::ProposalMismatch);
        }
        if evidence_bound.authority_epoch() != proposal.authority_epoch() {
            return Err(WitnessCheckpointReviewSetError::AuthorityEpochMismatch);
        }
        if trusted.project_policy() != proposal.project_policy() {
            return Err(WitnessCheckpointReviewSetError::ProjectPolicyMismatch);
        }
        if exact.reviewer() != claimed.reviewer() {
            return Err(WitnessCheckpointReviewSetError::ReviewerMismatch);
        }
        if exact.sequence() != claimed.sequence() {
            return Err(WitnessCheckpointReviewSetError::SequenceMismatch {
                reviewer: claimed.reviewer().clone(),
                claimed: claimed.sequence(),
                authenticated: exact.sequence(),
            });
        }

        let exact_id = exact.revision_id(claimed.revision().commitment().algorithm())?;
        if &exact_id != claimed.revision() {
            return Err(WitnessCheckpointReviewSetError::RevisionMismatch(
                claimed.reviewer().clone(),
            ));
        }
        if eligible.observed_at_unix_ms() > witnessed.quorum_observed_at_unix_ms() {
            return Err(
                WitnessCheckpointReviewSetError::RevisionObservedAfterCheckpoint {
                    reviewer: claimed.reviewer().clone(),
                    revision_observed_at_unix_ms: eligible.observed_at_unix_ms(),
                    checkpoint_observed_at_unix_ms: witnessed.quorum_observed_at_unix_ms(),
                },
            );
        }

        heads.push(CheckpointReviewHeadV1 {
            reviewer: claimed.reviewer().clone(),
            revision: exact_id,
            sequence: exact.sequence(),
            decision: exact.decision(),
            revision_observed_at_unix_ms: eligible.observed_at_unix_ms(),
            trusted_revision_evidence: trusted.evidence_commitment().clone(),
            verifier_identity: trusted.verifier_identity().clone(),
        });
    }

    if let Some((reviewer, _)) = by_reviewer.into_iter().next() {
        return Err(WitnessCheckpointReviewSetError::ExtraTrustedRevision(
            reviewer,
        ));
    }

    // Snapshot heads are already canonical by reviewer. Reassert the invariant
    // at the consuming boundary so a future snapshot refactor cannot silently
    // make aggregate evidence caller-order dependent.
    if heads.windows(2).any(|pair| pair[0].reviewer >= pair[1].reviewer) {
        return Err(WitnessCheckpointReviewSetError::NonCanonicalHeads);
    }

    let count = u32::try_from(heads.len()).map_err(|_| {
        WitnessCheckpointReviewSetError::CanonicalFieldTooLarge {
            field: "heads",
            len: heads.len(),
            max: u32::MAX as usize,
        }
    })?;
    let algorithm = witnessed.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(CHECKPOINT_REVIEW_SET_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, expected_proposal.commitment())?;
    push_digest(&mut out, proposal.authority_epoch())?;
    push_digest(&mut out, proposal.project_policy())?;
    push_digest(&mut out, proposal.repository_policy_state())?;
    push_digest(&mut out, snapshot.evidence_commitment())?;
    push_digest(&mut out, witnessed.evidence_commitment())?;
    push_digest(&mut out, witnessed.provider_namespace())?;
    push_digest(&mut out, witnessed.provider_checkpoint())?;
    out.extend_from_slice(&witnessed.quorum_observed_at_unix_ms().to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for head in &heads {
        push_digest(&mut out, head.reviewer.commitment())?;
        push_digest(&mut out, head.revision.commitment())?;
        out.extend_from_slice(&head.sequence.to_be_bytes());
        out.push(decision_code(head.decision));
        out.extend_from_slice(&head.revision_observed_at_unix_ms.to_be_bytes());
        push_digest(&mut out, &head.trusted_revision_evidence)?;
        push_digest(&mut out, &head.verifier_identity)?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(WitnessCheckpointReviewSetV1 {
        project: proposal.project().clone(),
        proposal: expected_proposal,
        authority_epoch: proposal.authority_epoch().clone(),
        project_policy: proposal.project_policy().clone(),
        repository_policy_state: proposal.repository_policy_state().clone(),
        snapshot_evidence: snapshot.evidence_commitment().clone(),
        witness_checkpoint_evidence: witnessed.evidence_commitment().clone(),
        provider_namespace: witnessed.provider_namespace().clone(),
        provider_checkpoint: witnessed.provider_checkpoint().clone(),
        checkpoint_observed_at_unix_ms: witnessed.quorum_observed_at_unix_ms(),
        heads,
        evidence_commitment,
    })
}

fn trusted_exact_revision(
    trusted: &ProjectPolicyTrustedReviewRevisionV1,
) -> &mycelix_forge_review_state::ReviewRevisionV1 {
    trusted.revision().revision().revision().revision()
}

const fn decision_code(decision: ReviewRevisionDecision) -> u8 {
    match decision {
        ReviewRevisionDecision::Approve => 1,
        ReviewRevisionDecision::RequestChanges => 2,
        ReviewRevisionDecision::Withdraw => 3,
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), WitnessCheckpointReviewSetError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), WitnessCheckpointReviewSetError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), WitnessCheckpointReviewSetError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        WitnessCheckpointReviewSetError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Exact checkpoint-membership join failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum WitnessCheckpointReviewSetError {
    /// Project contexts disagree.
    #[error("witness checkpoint review-set project mismatch")]
    ProjectMismatch,
    /// Proposal identities disagree.
    #[error("witness checkpoint review-set proposal mismatch")]
    ProposalMismatch,
    /// Authority epoch commitments disagree.
    #[error("witness checkpoint review-set authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Project-policy commitments disagree.
    #[error("witness checkpoint review-set project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Repository-policy commitments disagree.
    #[error("witness checkpoint review-set repository-policy mismatch")]
    RepositoryPolicyMismatch,
    /// Witness theorem refers to another snapshot evidence object.
    #[error("witness-qualified checkpoint does not name supplied snapshot evidence")]
    SnapshotEvidenceMismatch,
    /// Collaboration-state provider namespaces disagree.
    #[error("witness-qualified checkpoint provider namespace mismatch")]
    ProviderNamespaceMismatch,
    /// Opaque provider checkpoints disagree.
    #[error("witness-qualified checkpoint provider checkpoint mismatch")]
    ProviderCheckpointMismatch,
    /// More than one trusted revision was supplied for one reviewer.
    #[error("duplicate project-policy-trusted review revision for reviewer {0}")]
    DuplicateTrustedRevision(PrincipalId),
    /// Snapshot names a reviewer for which no trusted revision was supplied.
    #[error("missing project-policy-trusted review revision for reviewer {0}")]
    MissingTrustedRevision(PrincipalId),
    /// A trusted revision was supplied for a reviewer not named by snapshot.
    #[error("extra project-policy-trusted review revision for reviewer {0}")]
    ExtraTrustedRevision(PrincipalId),
    /// Authenticated reviewer differs from snapshot reviewer.
    #[error("authenticated review revision reviewer mismatch")]
    ReviewerMismatch,
    /// Reviewer-local sequence differs from snapshot claim.
    #[error(
        "reviewer {reviewer} sequence mismatch: snapshot={claimed}, authenticated={authenticated}"
    )]
    SequenceMismatch {
        /// Reviewer whose sequence differs.
        reviewer: PrincipalId,
        /// Sequence named by snapshot.
        claimed: u64,
        /// Sequence in authenticated revision.
        authenticated: u64,
    },
    /// Exact authenticated revision id differs from snapshot head.
    #[error("authenticated review revision id mismatch for reviewer {0}")]
    RevisionMismatch(PrincipalId),
    /// A revision's structural observation time is later than the witness
    /// checkpoint's common observation time.
    #[error(
        "reviewer {reviewer} revision observed at {revision_observed_at_unix_ms} after checkpoint {checkpoint_observed_at_unix_ms}"
    )]
    RevisionObservedAfterCheckpoint {
        /// Reviewer whose revision has inconsistent chronology.
        reviewer: PrincipalId,
        /// FORGE-007D structural observation time.
        revision_observed_at_unix_ms: u64,
        /// FORGE-007F common checkpoint observation time.
        checkpoint_observed_at_unix_ms: u64,
    },
    /// Joined heads were unexpectedly noncanonical.
    #[error("checkpoint review heads are not in canonical reviewer order")]
    NonCanonicalHeads,
    /// Canonical field exceeded the v1 encoding bound.
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
    /// Review-revision identity/canonicalization failure.
    #[error(transparent)]
    ReviewState(#[from] ReviewStateError),
}

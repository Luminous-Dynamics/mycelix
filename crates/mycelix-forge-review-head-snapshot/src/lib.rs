// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-neutral review-head snapshot claims for Mycelix Forge.
//!
//! This crate deliberately separates three different statements:
//!
//! 1. a caller can name an exact set of reviewer-local review heads at one
//!    provider checkpoint;
//! 2. a provider can attach opaque evidence claiming how that snapshot was
//!    observed and whether it is complete for the proposal;
//! 3. a later concrete provider verifier must prove that completeness claim.
//!
//! This crate establishes only (1) and the exact structural binding for (2).
//! It never upgrades opaque provider evidence into proof that no omitted later
//! revision exists.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::PrincipalId;
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_state::ReviewRevisionId;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const REVIEW_HEAD_SNAPSHOT_DOMAIN_V1: &[u8] = b"mycelix-forge/review-head-snapshot/v1\0";
const REVIEW_HEAD_OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-head-completeness-observation/v1\0";
const REVIEW_HEAD_EVIDENCE_DOMAIN_V1: &[u8] =
    b"mycelix-forge/evidence-bound-review-head-snapshot/v1\0";
const MAX_REVIEW_HEADS_V1: usize = 65_535;

/// One reviewer-local head claimed by a review-head snapshot.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewHeadV1 {
    reviewer: PrincipalId,
    revision: ReviewRevisionId,
    sequence: u64,
}

impl ReviewHeadV1 {
    /// Construct one claimed reviewer-local head.
    pub fn new(reviewer: PrincipalId, revision: ReviewRevisionId, sequence: u64) -> Self {
        Self {
            reviewer,
            revision,
            sequence,
        }
    }

    /// Reviewer whose lineage this head belongs to.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Exact claimed head revision.
    pub fn revision(&self) -> &ReviewRevisionId {
        &self.revision
    }

    /// Reviewer-local sequence claimed for the head.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }
}

/// Scope asserted by opaque provider coverage evidence.
///
/// This enum records what a provider claims. It does **not** prove the claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewHeadCoverageClaimV1 {
    /// Provider reports only the set it observed and makes no completeness
    /// claim.
    ObservedSetOnly,
    /// Provider claims the snapshot includes every review lineage/head in its
    /// defined proposal-wide coverage domain.
    ClaimsCompleteForProposal,
}

impl ReviewHeadCoverageClaimV1 {
    const fn code(self) -> u8 {
        match self {
            Self::ObservedSetOnly => 1,
            Self::ClaimsCompleteForProposal => 2,
        }
    }
}

/// Exact provider-neutral claim naming review heads at one provider checkpoint.
///
/// The `provider_checkpoint` is an opaque commitment interpreted by a concrete
/// collaboration-state adapter. For Holochain this may commit to a bounded
/// source-chain/DHT observation context; other providers may use a different
/// checkpoint scheme.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewHeadSnapshotClaimV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    provider_checkpoint: Digest,
    heads: Vec<ReviewHeadV1>,
}

impl ReviewHeadSnapshotClaimV1 {
    /// Construct a canonical snapshot claim for one exact proposal.
    ///
    /// Heads are sorted by reviewer. Duplicate reviewer principals are rejected
    /// rather than silently collapsed.
    pub fn new(
        proposal: &ChangeProposal,
        provider_checkpoint: Digest,
        mut heads: Vec<ReviewHeadV1>,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ReviewHeadSnapshotError> {
        if heads.len() > MAX_REVIEW_HEADS_V1 {
            return Err(ReviewHeadSnapshotError::TooManyReviewHeads {
                actual: heads.len(),
                max: MAX_REVIEW_HEADS_V1,
            });
        }
        heads.sort_by(|a, b| a.reviewer.cmp(&b.reviewer));
        if let Some(pair) = heads
            .windows(2)
            .find(|pair| pair[0].reviewer == pair[1].reviewer)
        {
            return Err(ReviewHeadSnapshotError::DuplicateReviewer(
                pair[0].reviewer.clone(),
            ));
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project: proposal.project().clone(),
            proposal: proposal.proposal_id(commitment_algorithm)?,
            authority_epoch: proposal.authority_epoch().clone(),
            project_policy: proposal.project_policy().clone(),
            repository_policy_state: proposal.repository_policy_state().clone(),
            provider_checkpoint,
            heads,
        })
    }

    fn from_parts(
        project: ProjectIdentity,
        proposal: ChangeProposalId,
        authority_epoch: Digest,
        project_policy: Digest,
        repository_policy_state: Digest,
        provider_checkpoint: Digest,
        heads: Vec<ReviewHeadV1>,
    ) -> Result<Self, ReviewHeadSnapshotError> {
        if heads.len() > MAX_REVIEW_HEADS_V1 {
            return Err(ReviewHeadSnapshotError::TooManyReviewHeads {
                actual: heads.len(),
                max: MAX_REVIEW_HEADS_V1,
            });
        }
        if heads.windows(2).any(|pair| pair[0].reviewer >= pair[1].reviewer) {
            return Err(ReviewHeadSnapshotError::NonCanonicalReviewHeads);
        }
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            proposal,
            authority_epoch,
            project_policy,
            repository_policy_state,
            provider_checkpoint,
            heads,
        })
    }

    /// Project containing the proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal whose review state is claimed.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact proposal authority epoch commitment.
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

    /// Opaque concrete-provider checkpoint commitment.
    pub fn provider_checkpoint(&self) -> &Digest {
        &self.provider_checkpoint
    }

    /// Canonically ordered claimed reviewer heads.
    pub fn heads(&self) -> &[ReviewHeadV1] {
        &self.heads
    }

    /// Canonical bytes for this exact snapshot claim.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewHeadSnapshotError> {
        ensure_v1(self.version)?;
        if self.heads.windows(2).any(|pair| pair[0].reviewer >= pair[1].reviewer) {
            return Err(ReviewHeadSnapshotError::NonCanonicalReviewHeads);
        }

        let count = u32::try_from(self.heads.len()).map_err(|_| {
            ReviewHeadSnapshotError::CanonicalFieldTooLarge {
                field: "heads",
                len: self.heads.len(),
                max: u32::MAX as usize,
            }
        })?;

        let mut out = Vec::new();
        out.extend_from_slice(REVIEW_HEAD_SNAPSHOT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.project_policy)?;
        push_digest(&mut out, &self.repository_policy_state)?;
        push_digest(&mut out, &self.provider_checkpoint)?;
        out.extend_from_slice(&count.to_be_bytes());
        for head in &self.heads {
            push_digest(&mut out, head.reviewer.commitment())?;
            push_digest(&mut out, head.revision.commitment())?;
            out.extend_from_slice(&head.sequence.to_be_bytes());
        }
        Ok(out)
    }

    /// Stable identity of the exact snapshot claim.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ReviewHeadSnapshotError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ReviewHeadSnapshotClaimV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireSnapshot {
            version: ProtocolVersion,
            project: ProjectIdentity,
            proposal: ChangeProposalId,
            authority_epoch: Digest,
            project_policy: Digest,
            repository_policy_state: Digest,
            provider_checkpoint: Digest,
            heads: Vec<ReviewHeadV1>,
        }

        let wire = WireSnapshot::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_parts(
            wire.project,
            wire.proposal,
            wire.authority_epoch,
            wire.project_policy,
            wire.repository_policy_state,
            wire.provider_checkpoint,
            wire.heads,
        )
        .map_err(D::Error::custom)
    }
}

/// Raw provider observation about one exact review-head snapshot.
///
/// All evidence digests are opaque at this layer. In particular,
/// `ClaimsCompleteForProposal` is only a provider claim until a concrete
/// provider verifier establishes it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewHeadCompletenessObservationV1 {
    version: ProtocolVersion,
    snapshot_commitment: Digest,
    provider_namespace: Digest,
    coverage: ReviewHeadCoverageClaimV1,
    checkpoint_evidence: Digest,
    coverage_evidence: Digest,
    freshness_evidence: Digest,
}

impl ReviewHeadCompletenessObservationV1 {
    /// Construct an opaque provider observation.
    pub fn new(
        snapshot_commitment: Digest,
        provider_namespace: Digest,
        coverage: ReviewHeadCoverageClaimV1,
        checkpoint_evidence: Digest,
        coverage_evidence: Digest,
        freshness_evidence: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            snapshot_commitment,
            provider_namespace,
            coverage,
            checkpoint_evidence,
            coverage_evidence,
            freshness_evidence,
        }
    }

    /// Snapshot claim this observation names.
    pub fn snapshot_commitment(&self) -> &Digest {
        &self.snapshot_commitment
    }

    /// Concrete provider namespace interpreting the opaque evidence.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Scope the provider claims its coverage evidence establishes.
    pub const fn coverage(&self) -> ReviewHeadCoverageClaimV1 {
        self.coverage
    }

    /// Opaque evidence for the provider checkpoint.
    pub fn checkpoint_evidence(&self) -> &Digest {
        &self.checkpoint_evidence
    }

    /// Opaque evidence for claimed review-head coverage/completeness.
    pub fn coverage_evidence(&self) -> &Digest {
        &self.coverage_evidence
    }

    /// Opaque freshness/observation evidence.
    pub fn freshness_evidence(&self) -> &Digest {
        &self.freshness_evidence
    }

    /// Canonical bytes for the raw observation.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewHeadSnapshotError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(REVIEW_HEAD_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_digest(&mut out, &self.snapshot_commitment)?;
        push_digest(&mut out, &self.provider_namespace)?;
        out.push(self.coverage.code());
        push_digest(&mut out, &self.checkpoint_evidence)?;
        push_digest(&mut out, &self.coverage_evidence)?;
        push_digest(&mut out, &self.freshness_evidence)?;
        Ok(out)
    }

    /// Stable identity of the exact provider observation.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ReviewHeadSnapshotError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Positive structural result binding opaque provider evidence to one exact
/// review-head snapshot claim.
///
/// This type deliberately does **not** say `Complete` or `Current`. Opaque
/// evidence can be cross-linked perfectly while still being false or
/// incomplete.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct EvidenceBoundReviewHeadSnapshotV1 {
    snapshot: ReviewHeadSnapshotClaimV1,
    observation: ReviewHeadCompletenessObservationV1,
    evidence_commitment: Digest,
}

impl EvidenceBoundReviewHeadSnapshotV1 {
    /// Exact snapshot claim whose opaque provider evidence is bound.
    pub fn snapshot(&self) -> &ReviewHeadSnapshotClaimV1 {
        &self.snapshot
    }

    /// Raw provider completeness/observation claim.
    pub fn observation(&self) -> &ReviewHeadCompletenessObservationV1 {
        &self.observation
    }

    /// Aggregate structural evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Structurally bind opaque provider evidence to one exact snapshot claim.
pub fn bind_review_head_snapshot_observation_v1(
    snapshot: ReviewHeadSnapshotClaimV1,
    observation: ReviewHeadCompletenessObservationV1,
) -> Result<EvidenceBoundReviewHeadSnapshotV1, ReviewHeadSnapshotError> {
    let expected = snapshot.digest(observation.snapshot_commitment().algorithm())?;
    if observation.snapshot_commitment() != &expected {
        return Err(ReviewHeadSnapshotError::ObservationSnapshotMismatch);
    }

    let algorithm = observation.snapshot_commitment().algorithm();
    let observation_digest = observation.digest(algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(REVIEW_HEAD_EVIDENCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, &expected)?;
    push_digest(&mut out, &observation_digest)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(EvidenceBoundReviewHeadSnapshotV1 {
        snapshot,
        observation,
        evidence_commitment,
    })
}

/// Review-head snapshot validation/canonicalization failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewHeadSnapshotError {
    /// Unsupported Forge protocol version.
    #[error("unsupported Forge review-head snapshot protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Snapshot contains the same reviewer more than once.
    #[error("duplicate reviewer in review-head snapshot: {0}")]
    DuplicateReviewer(PrincipalId),
    /// Serialized reviewer heads are not in strict canonical order.
    #[error("review-head snapshot reviewer ordering is non-canonical or duplicated")]
    NonCanonicalReviewHeads,
    /// Defensive v1 head-count bound exceeded.
    #[error("too many review heads: {actual} > {max}")]
    TooManyReviewHeads {
        /// Observed count.
        actual: usize,
        /// Maximum allowed count.
        max: usize,
    },
    /// Provider observation names another snapshot claim.
    #[error("review-head provider observation does not name supplied snapshot")]
    ObservationSnapshotMismatch,
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
    /// Proposal identity/canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ReviewHeadSnapshotError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewHeadSnapshotError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewHeadSnapshotError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewHeadSnapshotError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewHeadSnapshotError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ReviewHeadSnapshotError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authority::{
        AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant,
    };
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review_state::{ReviewRevisionDecision, ReviewRevisionV1};

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

    fn proposal(proposed: u8) -> ChangeProposal {
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(digest(0x21), digest(0x22))],
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
            valid_until_unix_ms: None,
            grants: vec![PrincipalGrant::new(
                principal(0x10),
                [Capability::ManageAuthority],
            )
            .unwrap()],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap();
        let repository_policy =
            RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        ChangeProposal::new(
            principal(0x30),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x40; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![proposed; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x42; 20]).unwrap(),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn head(proposal: &ChangeProposal, reviewer: u8, decision: ReviewRevisionDecision) -> ReviewHeadV1 {
        let revision = ReviewRevisionV1::genesis(
            proposal,
            principal(reviewer),
            decision,
            digest(reviewer.wrapping_add(1)),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        ReviewHeadV1::new(
            principal(reviewer),
            revision.revision_id(DigestAlgorithm::Sha256).unwrap(),
            revision.sequence(),
        )
    }

    #[test]
    fn public_construction_canonicalizes_head_order() {
        let proposal = proposal(0x41);
        let a = head(&proposal, 0x20, ReviewRevisionDecision::Approve);
        let b = head(&proposal, 0x21, ReviewRevisionDecision::RequestChanges);
        let forward = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![a.clone(), b.clone()],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let reverse = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![b, a],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_eq!(forward, reverse);
        assert_eq!(
            forward.digest(DigestAlgorithm::Sha256).unwrap(),
            reverse.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn duplicate_reviewer_is_rejected_not_deduplicated() {
        let proposal = proposal(0x41);
        let a = head(&proposal, 0x20, ReviewRevisionDecision::Approve);
        assert_eq!(
            ReviewHeadSnapshotClaimV1::new(
                &proposal,
                digest(0x60),
                vec![a.clone(), a],
                DigestAlgorithm::Sha256,
            )
            .unwrap_err(),
            ReviewHeadSnapshotError::DuplicateReviewer(principal(0x20))
        );
    }

    #[test]
    fn changing_head_or_sequence_changes_snapshot_identity() {
        let proposal = proposal(0x41);
        let original = head(&proposal, 0x20, ReviewRevisionDecision::Approve);
        let changed_revision = head(&proposal, 0x20, ReviewRevisionDecision::Withdraw);
        let changed_sequence = ReviewHeadV1::new(
            original.reviewer().clone(),
            original.revision().clone(),
            1,
        );
        let a = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![original],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let b = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![changed_revision],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let c = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![changed_sequence],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(a.digest(DigestAlgorithm::Sha256).unwrap(), b.digest(DigestAlgorithm::Sha256).unwrap());
        assert_ne!(a.digest(DigestAlgorithm::Sha256).unwrap(), c.digest(DigestAlgorithm::Sha256).unwrap());
    }

    #[test]
    fn observation_must_name_exact_snapshot() {
        let proposal = proposal(0x41);
        let snapshot = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![head(&proposal, 0x20, ReviewRevisionDecision::Approve)],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let observation = ReviewHeadCompletenessObservationV1::new(
            digest(0x99),
            digest(0x70),
            ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal,
            digest(0x71),
            digest(0x72),
            digest(0x73),
        );
        assert_eq!(
            bind_review_head_snapshot_observation_v1(snapshot, observation).unwrap_err(),
            ReviewHeadSnapshotError::ObservationSnapshotMismatch
        );
    }

    #[test]
    fn coverage_claim_changes_observation_and_evidence_identity() {
        let proposal = proposal(0x41);
        let snapshot = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![head(&proposal, 0x20, ReviewRevisionDecision::Approve)],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let snapshot_id = snapshot.digest(DigestAlgorithm::Sha256).unwrap();
        let observed = ReviewHeadCompletenessObservationV1::new(
            snapshot_id.clone(),
            digest(0x70),
            ReviewHeadCoverageClaimV1::ObservedSetOnly,
            digest(0x71),
            digest(0x72),
            digest(0x73),
        );
        let claimed_complete = ReviewHeadCompletenessObservationV1::new(
            snapshot_id,
            digest(0x70),
            ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal,
            digest(0x71),
            digest(0x72),
            digest(0x73),
        );
        assert_ne!(
            observed.digest(DigestAlgorithm::Sha256).unwrap(),
            claimed_complete.digest(DigestAlgorithm::Sha256).unwrap()
        );
        let a = bind_review_head_snapshot_observation_v1(snapshot.clone(), observed).unwrap();
        let b = bind_review_head_snapshot_observation_v1(snapshot, claimed_complete).unwrap();
        assert_ne!(a.evidence_commitment(), b.evidence_commitment());
    }

    #[test]
    fn opaque_complete_claim_can_bind_structurally_without_proving_completeness() {
        let proposal = proposal(0x41);
        let snapshot = ReviewHeadSnapshotClaimV1::new(
            &proposal,
            digest(0x60),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let observation = ReviewHeadCompletenessObservationV1::new(
            snapshot.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0x70),
            ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal,
            digest(0x01),
            digest(0x02),
            digest(0x03),
        );
        let bound = bind_review_head_snapshot_observation_v1(snapshot, observation).unwrap();
        assert_eq!(
            bound.observation().coverage(),
            ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal
        );
        // The three provider evidence digests above are arbitrary. Success proves
        // only exact structural binding, never actual global completeness.
    }

    #[test]
    fn proposal_change_changes_snapshot_identity_even_with_same_heads_and_checkpoint() {
        let a_proposal = proposal(0x41);
        let b_proposal = proposal(0x43);
        let a = ReviewHeadSnapshotClaimV1::new(
            &a_proposal,
            digest(0x60),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let b = ReviewHeadSnapshotClaimV1::new(
            &b_proposal,
            digest(0x60),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(
            a.digest(DigestAlgorithm::Sha256).unwrap(),
            b.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }
}

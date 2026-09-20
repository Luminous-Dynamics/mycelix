// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Lineage-aware review-state revisions for Mycelix Forge.
//!
//! Immutable review attestations are sufficient to prove that a reviewer made
//! an exact statement, but they are not sufficient to prove which statement is
//! currently in force. A caller can always omit a later conflicting review.
//!
//! This crate introduces an explicit per-reviewer revision lineage without
//! overclaiming global currentness:
//!
//! ```text
//! ReviewRevisionV1(sequence N, previous revision, decision, context)
//!     ↓
//! exact ReviewRevisionId
//!     ↓
//! supplied complete local chain can be validated
//!     != globally current review head
//! ```
//!
//! A future append-only collaboration-state provider must prove that a
//! validated supplied head is the current complete head before merge authority
//! can rely on it.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::PrincipalId;
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const REVIEW_REVISION_DOMAIN_V1: &[u8] = b"mycelix-forge/review-revision/v1\0";
const SUPPLIED_CHAIN_DOMAIN_V1: &[u8] = b"mycelix-forge/supplied-review-revision-chain/v1\0";
const MAX_SUPPLIED_REVISIONS_V1: usize = 4096;

/// Security-significant review state represented by one revision.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewRevisionDecision {
    /// Reviewer approves the exact proposal at this revision.
    Approve,
    /// Reviewer requests changes to the exact proposal at this revision.
    RequestChanges,
    /// Reviewer explicitly withdraws their prior review state without
    /// expressing a replacement approval or request-changes decision.
    Withdraw,
}

impl ReviewRevisionDecision {
    const fn code(self) -> u8 {
        match self {
            Self::Approve => 1,
            Self::RequestChanges => 2,
            Self::Withdraw => 3,
        }
    }
}

/// Stable identifier for one exact review revision.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ReviewRevisionId(Digest);

impl ReviewRevisionId {
    /// Construct from an already-derived revision commitment.
    pub fn new(commitment: Digest) -> Self {
        Self(commitment)
    }

    /// Algorithm-qualified revision commitment.
    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

/// One immutable revision in a single reviewer's review-state lineage for one
/// exact proposal.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewRevisionV1 {
    version: ProtocolVersion,
    proposal: ChangeProposalId,
    reviewer: PrincipalId,
    sequence: u64,
    previous: Option<ReviewRevisionId>,
    decision: ReviewRevisionDecision,
    review_context: Digest,
}

impl ReviewRevisionV1 {
    /// Construct sequence-zero review state for one exact proposal/reviewer.
    pub fn genesis(
        proposal: &ChangeProposal,
        reviewer: PrincipalId,
        decision: ReviewRevisionDecision,
        review_context: Digest,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ReviewStateError> {
        Self::from_parts(
            proposal.proposal_id(commitment_algorithm)?,
            reviewer,
            0,
            None,
            decision,
            review_context,
        )
    }

    /// Construct the exact next revision of an existing review lineage.
    ///
    /// The proposal and reviewer are inherited from `previous`; callers cannot
    /// switch reviewer or proposal while retaining predecessor continuity.
    pub fn successor(
        previous: &Self,
        decision: ReviewRevisionDecision,
        review_context: Digest,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ReviewStateError> {
        let sequence = previous
            .sequence
            .checked_add(1)
            .ok_or(ReviewStateError::SequenceOverflow)?;
        let previous_id = previous.revision_id(commitment_algorithm)?;
        Self::from_parts(
            previous.proposal.clone(),
            previous.reviewer.clone(),
            sequence,
            Some(previous_id),
            decision,
            review_context,
        )
    }

    fn from_parts(
        proposal: ChangeProposalId,
        reviewer: PrincipalId,
        sequence: u64,
        previous: Option<ReviewRevisionId>,
        decision: ReviewRevisionDecision,
        review_context: Digest,
    ) -> Result<Self, ReviewStateError> {
        validate_sequence_shape(sequence, previous.as_ref())?;
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            proposal,
            reviewer,
            sequence,
            previous,
            decision,
            review_context,
        })
    }

    /// Exact proposal whose review state this lineage describes.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Reviewer principal owning this lineage.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Monotonic reviewer-local revision sequence.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Exact predecessor revision, absent only for sequence zero.
    pub fn previous(&self) -> Option<&ReviewRevisionId> {
        self.previous.as_ref()
    }

    /// Security-significant state at this revision.
    pub const fn decision(&self) -> ReviewRevisionDecision {
        self.decision
    }

    /// Immutable review rationale/checklist/result commitment for this state.
    pub fn review_context(&self) -> &Digest {
        &self.review_context
    }

    /// Canonical v1 bytes for this exact revision.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewStateError> {
        ensure_v1(self.version)?;
        validate_sequence_shape(self.sequence, self.previous.as_ref())?;

        let mut out = Vec::new();
        out.extend_from_slice(REVIEW_REVISION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, self.reviewer.commitment())?;
        out.extend_from_slice(&self.sequence.to_be_bytes());
        push_optional_revision_id(&mut out, self.previous.as_ref())?;
        out.push(self.decision.code());
        push_digest(&mut out, &self.review_context)?;
        Ok(out)
    }

    /// Stable identity of this exact review revision.
    pub fn revision_id(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<ReviewRevisionId, ReviewStateError> {
        Ok(ReviewRevisionId::new(Digest::of_bytes(
            algorithm,
            &self.canonical_bytes()?,
        )))
    }
}

impl<'de> Deserialize<'de> for ReviewRevisionV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireRevision {
            version: ProtocolVersion,
            proposal: ChangeProposalId,
            reviewer: PrincipalId,
            sequence: u64,
            previous: Option<ReviewRevisionId>,
            decision: ReviewRevisionDecision,
            review_context: Digest,
        }

        let wire = WireRevision::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_parts(
            wire.proposal,
            wire.reviewer,
            wire.sequence,
            wire.previous,
            wire.decision,
            wire.review_context,
        )
        .map_err(D::Error::custom)
    }
}

/// Positive result proving only that the supplied revision list is one exact,
/// internally complete predecessor chain from sequence zero to the supplied
/// head.
///
/// It does **not** prove that the head is globally current or that no later
/// revision exists outside the supplied list.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ValidatedSuppliedReviewRevisionChainV1 {
    proposal: ChangeProposalId,
    reviewer: PrincipalId,
    head: ReviewRevisionId,
    head_sequence: u64,
    revision_count: u32,
    chain_commitment: Digest,
}

impl ValidatedSuppliedReviewRevisionChainV1 {
    /// Exact proposal shared by every supplied revision.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact reviewer shared by every supplied revision.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Exact final revision in the supplied chain.
    pub fn head(&self) -> &ReviewRevisionId {
        &self.head
    }

    /// Reviewer-local sequence of the supplied head.
    pub const fn head_sequence(&self) -> u64 {
        self.head_sequence
    }

    /// Number of revisions validated in the supplied chain.
    pub const fn revision_count(&self) -> u32 {
        self.revision_count
    }

    /// Aggregate commitment to the exact ordered supplied chain.
    pub fn chain_commitment(&self) -> &Digest {
        &self.chain_commitment
    }
}

/// Validate one fully supplied reviewer-local revision chain.
///
/// The first revision must be sequence zero. Every later element must increment
/// sequence by exactly one, retain proposal/reviewer identity, and name the
/// exact preceding revision id under `commitment_algorithm`.
pub fn validate_supplied_review_revision_chain_v1(
    revisions: &[ReviewRevisionV1],
    commitment_algorithm: DigestAlgorithm,
) -> Result<ValidatedSuppliedReviewRevisionChainV1, ReviewStateError> {
    if revisions.is_empty() {
        return Err(ReviewStateError::EmptySuppliedChain);
    }
    if revisions.len() > MAX_SUPPLIED_REVISIONS_V1 {
        return Err(ReviewStateError::TooManySuppliedRevisions {
            actual: revisions.len(),
            max: MAX_SUPPLIED_REVISIONS_V1,
        });
    }

    let first = &revisions[0];
    if first.sequence != 0 || first.previous.is_some() {
        return Err(ReviewStateError::ChainDoesNotStartAtGenesis);
    }
    let proposal = first.proposal.clone();
    let reviewer = first.reviewer.clone();

    let mut ids = Vec::with_capacity(revisions.len());
    ids.push(first.revision_id(commitment_algorithm)?);

    for (index, pair) in revisions.windows(2).enumerate() {
        let previous = &pair[0];
        let current = &pair[1];
        if current.proposal != proposal {
            return Err(ReviewStateError::ChainProposalMismatch { index: index + 1 });
        }
        if current.reviewer != reviewer {
            return Err(ReviewStateError::ChainReviewerMismatch { index: index + 1 });
        }
        let expected_sequence = previous
            .sequence
            .checked_add(1)
            .ok_or(ReviewStateError::SequenceOverflow)?;
        if current.sequence != expected_sequence {
            return Err(ReviewStateError::NonContiguousSequence {
                index: index + 1,
                expected: expected_sequence,
                actual: current.sequence,
            });
        }
        let expected_previous = previous.revision_id(commitment_algorithm)?;
        if current.previous.as_ref() != Some(&expected_previous) {
            return Err(ReviewStateError::PredecessorMismatch { index: index + 1 });
        }
        ids.push(current.revision_id(commitment_algorithm)?);
    }

    let revision_count = u32::try_from(revisions.len()).map_err(|_| {
        ReviewStateError::CanonicalFieldTooLarge {
            field: "revisions",
            len: revisions.len(),
            max: u32::MAX as usize,
        }
    })?;
    let head_revision = revisions.last().ok_or(ReviewStateError::EmptySuppliedChain)?;
    let head = ids.last().cloned().ok_or(ReviewStateError::EmptySuppliedChain)?;

    let mut out = Vec::new();
    out.extend_from_slice(SUPPLIED_CHAIN_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, proposal.commitment())?;
    push_digest(&mut out, reviewer.commitment())?;
    out.extend_from_slice(&revision_count.to_be_bytes());
    for id in &ids {
        push_digest(&mut out, id.commitment())?;
    }
    let chain_commitment = Digest::of_bytes(commitment_algorithm, &out);

    Ok(ValidatedSuppliedReviewRevisionChainV1 {
        proposal,
        reviewer,
        head,
        head_sequence: head_revision.sequence,
        revision_count,
        chain_commitment,
    })
}

fn validate_sequence_shape(
    sequence: u64,
    previous: Option<&ReviewRevisionId>,
) -> Result<(), ReviewStateError> {
    match (sequence, previous.is_some()) {
        (0, false) => Ok(()),
        (0, true) => Err(ReviewStateError::GenesisHasPrevious),
        (_, false) => Err(ReviewStateError::NonGenesisMissingPrevious),
        (_, true) => Ok(()),
    }
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ReviewStateError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewStateError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_optional_revision_id(
    out: &mut Vec<u8>,
    revision: Option<&ReviewRevisionId>,
) -> Result<(), ReviewStateError> {
    match revision {
        None => out.push(0),
        Some(revision) => {
            out.push(1);
            push_digest(out, revision.commitment())?;
        }
    }
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewStateError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewStateError> {
    let len = u32::try_from(bytes.len()).map_err(|_| ReviewStateError::CanonicalFieldTooLarge {
        field,
        len: bytes.len(),
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Review-revision validation/canonicalization failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewStateError {
    /// Unsupported review-state protocol version.
    #[error("unsupported Forge review-state protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Sequence zero cannot name a predecessor.
    #[error("genesis review revision cannot name a predecessor")]
    GenesisHasPrevious,
    /// Non-genesis revision must name its exact predecessor.
    #[error("non-genesis review revision is missing its predecessor")]
    NonGenesisMissingPrevious,
    /// Reviewer-local sequence overflowed.
    #[error("review revision sequence overflow")]
    SequenceOverflow,
    /// Supplied validation set is empty.
    #[error("supplied review revision chain is empty")]
    EmptySuppliedChain,
    /// Supplied validation set exceeded the v1 defensive bound.
    #[error("too many supplied review revisions: {actual} > {max}")]
    TooManySuppliedRevisions {
        /// Supplied count.
        actual: usize,
        /// Maximum v1 count.
        max: usize,
    },
    /// First supplied revision is not exact sequence-zero genesis.
    #[error("supplied review revision chain does not start at genesis")]
    ChainDoesNotStartAtGenesis,
    /// Later revision changes immutable proposal identity.
    #[error("review revision at index {index} changes proposal identity")]
    ChainProposalMismatch {
        /// Zero-based failing index.
        index: usize,
    },
    /// Later revision changes immutable reviewer identity.
    #[error("review revision at index {index} changes reviewer identity")]
    ChainReviewerMismatch {
        /// Zero-based failing index.
        index: usize,
    },
    /// Sequence does not increment by exactly one.
    #[error("review revision at index {index} has sequence {actual}, expected {expected}")]
    NonContiguousSequence {
        /// Zero-based failing index.
        index: usize,
        /// Required sequence.
        expected: u64,
        /// Supplied sequence.
        actual: u64,
    },
    /// Predecessor id does not equal the exact prior supplied revision.
    #[error("review revision at index {index} does not name the exact prior revision")]
    PredecessorMismatch {
        /// Zero-based failing index.
        index: usize,
    },
    /// Canonical field exceeded v1 length bound.
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authority::{
        AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant,
    };
    use mycelix_forge_core::{ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
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

    fn proposal(proposed_revision: u8) -> ChangeProposal {
        let project = project();
        let authority = AuthorityEpoch::new(AuthorityEpochParts {
            project: project.clone(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: None,
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
            ],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap();
        let trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![TrustedProviderVerifierV1::new(digest(0x20), digest(0x21))],
        )
        .unwrap();
        let project_policy = ProjectPolicyStateV1::new(
            project.clone(),
            0,
            None,
            &trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let repository_policy =
            RepositoryPolicyState::new(project, 0, None, digest(0x31)).unwrap();
        ChangeProposal::new(
            principal(0x30),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x40; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![proposed_revision; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x42; 20]).unwrap(),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    #[test]
    fn successor_binds_exact_predecessor_and_preserves_reviewer_and_proposal() {
        let proposal = proposal(0x41);
        let genesis = ReviewRevisionV1::genesis(
            &proposal,
            principal(0x60),
            ReviewRevisionDecision::RequestChanges,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let successor = ReviewRevisionV1::successor(
            &genesis,
            ReviewRevisionDecision::Approve,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();

        assert_eq!(successor.sequence(), 1);
        assert_eq!(successor.reviewer(), genesis.reviewer());
        assert_eq!(successor.proposal(), genesis.proposal());
        assert_eq!(
            successor.previous(),
            Some(&genesis.revision_id(DigestAlgorithm::Sha256).unwrap())
        );
    }

    #[test]
    fn decision_context_and_predecessor_change_revision_identity() {
        let proposal = proposal(0x41);
        let genesis = ReviewRevisionV1::genesis(
            &proposal,
            principal(0x60),
            ReviewRevisionDecision::RequestChanges,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let approve = ReviewRevisionV1::successor(
            &genesis,
            ReviewRevisionDecision::Approve,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let withdraw = ReviewRevisionV1::successor(
            &genesis,
            ReviewRevisionDecision::Withdraw,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let context_changed = ReviewRevisionV1::successor(
            &genesis,
            ReviewRevisionDecision::Approve,
            digest(0x72),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(
            approve.revision_id(DigestAlgorithm::Sha256).unwrap(),
            withdraw.revision_id(DigestAlgorithm::Sha256).unwrap()
        );
        assert_ne!(
            approve.revision_id(DigestAlgorithm::Sha256).unwrap(),
            context_changed.revision_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn exact_supplied_chain_validates_and_commits_ordered_history() {
        let proposal = proposal(0x41);
        let r0 = ReviewRevisionV1::genesis(
            &proposal,
            principal(0x60),
            ReviewRevisionDecision::RequestChanges,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let r1 = ReviewRevisionV1::successor(
            &r0,
            ReviewRevisionDecision::Approve,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let r2 = ReviewRevisionV1::successor(
            &r1,
            ReviewRevisionDecision::Withdraw,
            digest(0x72),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let chain = validate_supplied_review_revision_chain_v1(
            &[r0.clone(), r1.clone(), r2.clone()],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_eq!(chain.revision_count(), 3);
        assert_eq!(chain.head_sequence(), 2);
        assert_eq!(
            chain.head(),
            &r2.revision_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn omitted_middle_revision_breaks_predecessor_continuity() {
        let proposal = proposal(0x41);
        let r0 = ReviewRevisionV1::genesis(
            &proposal,
            principal(0x60),
            ReviewRevisionDecision::RequestChanges,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let r1 = ReviewRevisionV1::successor(
            &r0,
            ReviewRevisionDecision::Approve,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let r2 = ReviewRevisionV1::successor(
            &r1,
            ReviewRevisionDecision::Withdraw,
            digest(0x72),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert!(matches!(
            validate_supplied_review_revision_chain_v1(
                &[r0, r2],
                DigestAlgorithm::Sha256,
            ),
            Err(ReviewStateError::NonContiguousSequence { .. })
                | Err(ReviewStateError::PredecessorMismatch { .. })
        ));
    }

    #[test]
    fn reordered_chain_is_rejected() {
        let proposal = proposal(0x41);
        let r0 = ReviewRevisionV1::genesis(
            &proposal,
            principal(0x60),
            ReviewRevisionDecision::Approve,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let r1 = ReviewRevisionV1::successor(
            &r0,
            ReviewRevisionDecision::RequestChanges,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_eq!(
            validate_supplied_review_revision_chain_v1(
                &[r1, r0],
                DigestAlgorithm::Sha256,
            )
            .unwrap_err(),
            ReviewStateError::ChainDoesNotStartAtGenesis
        );
    }

    #[test]
    fn serde_rejects_non_genesis_without_previous() {
        #[derive(Serialize)]
        struct WireRevision {
            version: ProtocolVersion,
            proposal: ChangeProposalId,
            reviewer: PrincipalId,
            sequence: u64,
            previous: Option<ReviewRevisionId>,
            decision: ReviewRevisionDecision,
            review_context: Digest,
        }
        let proposal = proposal(0x41);
        let wire = WireRevision {
            version: ProtocolVersion::CURRENT,
            proposal: proposal.proposal_id(DigestAlgorithm::Sha256).unwrap(),
            reviewer: principal(0x60),
            sequence: 1,
            previous: None,
            decision: ReviewRevisionDecision::Approve,
            review_context: digest(0x70),
        };
        let encoded = serde_json::to_vec(&wire).unwrap();
        assert!(serde_json::from_slice::<ReviewRevisionV1>(&encoded).is_err());
    }

    #[test]
    fn proposal_mutation_starts_a_distinct_review_lineage() {
        let a = proposal(0x41);
        let b = proposal(0x43);
        let a0 = ReviewRevisionV1::genesis(
            &a,
            principal(0x60),
            ReviewRevisionDecision::Approve,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let b0 = ReviewRevisionV1::genesis(
            &b,
            principal(0x60),
            ReviewRevisionDecision::Approve,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(a0.proposal(), b0.proposal());
        assert_ne!(
            a0.revision_id(DigestAlgorithm::Sha256).unwrap(),
            b0.revision_id(DigestAlgorithm::Sha256).unwrap()
        );
    }
}

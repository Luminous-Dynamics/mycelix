// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Complete review-census and conflict-free approval evidence for Mycelix Forge.
//!
//! FORGE-007B proves a positive approval quorum over the reviews a caller
//! supplies. This crate handles the separate omission problem: a named review
//! source/verifier must positively qualify that a canonical census is complete
//! for one exact collaboration-state root before Forge can infer that no
//! opposing security-significant review was omitted.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::PrincipalId;
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review::{ReviewDecision, StructurallyAuthorizedReview};
use mycelix_forge_review_quorum::QualifiedApprovalQuorum;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

const REVIEW_CENSUS_DOMAIN_V1: &[u8] = b"mycelix-forge/review-census/v1\0";
const COMPLETENESS_OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-completeness-observation/v1\0";
const COMPLETE_CENSUS_DOMAIN_V1: &[u8] = b"mycelix-forge/qualified-complete-review-census/v1\0";
const CONFLICT_FREE_APPROVAL_DOMAIN_V1: &[u8] =
    b"mycelix-forge/conflict-free-approval-state/v1\0";
const MAX_ADAPTER_FIELD_LEN: usize = 128;
const MAX_REVIEWS_V1: usize = 4096;

/// One security-significant review in a canonical census.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewCensusEntry {
    reviewer: PrincipalId,
    decision: ReviewDecision,
    observed_at_unix_ms: u64,
    review_evidence: Digest,
}

impl ReviewCensusEntry {
    /// Reviewer principal represented by this entry.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Security-significant review decision.
    pub const fn decision(&self) -> ReviewDecision {
        self.decision
    }

    /// Observation time already bound by FORGE-007 structural review evidence.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact FORGE-007 structural review evidence commitment.
    pub fn review_evidence(&self) -> &Digest {
        &self.review_evidence
    }
}

/// Canonical supplied review set for one exact proposal and source-state root.
///
/// This type is **not** a completeness claim. It only normalizes and binds the
/// supplied reviews. Completeness requires [`QualifiedCompleteReviewCensus`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewCensus {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    repository_policy_state: Digest,
    source_state: Digest,
    entries: Vec<ReviewCensusEntry>,
}

impl ReviewCensus {
    /// Build a canonical v1 census from supplied structurally authorized reviews.
    ///
    /// Protocol v1 refuses more than one security-significant review per
    /// principal. There is no implicit timestamp-based supersession rule.
    pub fn new(
        proposal: &ChangeProposal,
        source_state: Digest,
        reviews: &[StructurallyAuthorizedReview],
        proposal_algorithm: DigestAlgorithm,
    ) -> Result<Self, ReviewCensusError> {
        if reviews.len() > MAX_REVIEWS_V1 {
            return Err(ReviewCensusError::TooManyReviews {
                actual: reviews.len(),
                max: MAX_REVIEWS_V1,
            });
        }

        let proposal_id = proposal.proposal_id(proposal_algorithm)?;
        let mut seen = BTreeSet::new();
        let mut entries = Vec::with_capacity(reviews.len());

        for review in reviews {
            let authenticated = review.authenticated_review();
            let statement = authenticated.statement();

            if authenticated.project() != proposal.project() {
                return Err(ReviewCensusError::ProjectMismatch);
            }
            if statement.proposal() != &proposal_id {
                return Err(ReviewCensusError::ProposalMismatch);
            }
            if statement.authority_epoch() != proposal.authority_epoch() {
                return Err(ReviewCensusError::AuthorityEpochMismatch);
            }
            if statement.repository_policy_state() != proposal.repository_policy_state() {
                return Err(ReviewCensusError::RepositoryPolicyMismatch);
            }
            if !seen.insert(statement.reviewer().clone()) {
                return Err(ReviewCensusError::AmbiguousReviewer(
                    statement.reviewer().clone(),
                ));
            }

            entries.push(ReviewCensusEntry {
                reviewer: statement.reviewer().clone(),
                decision: statement.decision(),
                observed_at_unix_ms: review.observed_at_unix_ms(),
                review_evidence: review.evidence_commitment().clone(),
            });
        }

        entries.sort_by(|a, b| a.reviewer.cmp(&b.reviewer));

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project: proposal.project().clone(),
            proposal: proposal_id,
            authority_epoch: proposal.authority_epoch().clone(),
            repository_policy_state: proposal.repository_policy_state().clone(),
            source_state,
            entries,
        })
    }

    /// Project containing the proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal represented by this census.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch inherited from the proposal.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact repository-policy state inherited from the proposal.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact collaboration/source-state root against which completeness is evaluated.
    pub fn source_state(&self) -> &Digest {
        &self.source_state
    }

    /// Canonically sorted security-significant review entries.
    pub fn entries(&self) -> &[ReviewCensusEntry] {
        &self.entries
    }

    /// Canonical v1 bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewCensusError> {
        ensure_v1(self.version)?;
        let count = u32::try_from(self.entries.len()).map_err(|_| {
            ReviewCensusError::CanonicalFieldTooLarge {
                field: "entries",
                len: self.entries.len(),
                max: u32::MAX as usize,
            }
        })?;

        let mut out = Vec::new();
        out.extend_from_slice(REVIEW_CENSUS_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.repository_policy_state)?;
        push_digest(&mut out, &self.source_state)?;
        out.extend_from_slice(&count.to_be_bytes());
        for entry in &self.entries {
            push_digest(&mut out, entry.reviewer.commitment())?;
            out.push(review_decision_code(entry.decision));
            out.extend_from_slice(&entry.observed_at_unix_ms.to_be_bytes());
            push_digest(&mut out, &entry.review_evidence)?;
        }
        Ok(out)
    }

    /// Exact census commitment.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ReviewCensusError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Stable identity of a review-completeness adapter/verifier implementation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewSourceAdapter {
    implementation: String,
    version: String,
}

impl ReviewSourceAdapter {
    /// Construct a bounded non-empty adapter identity.
    pub fn new(
        implementation: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ReviewCensusError> {
        let implementation = implementation.into();
        let version = version.into();
        validate_adapter_field("implementation", &implementation)?;
        validate_adapter_field("version", &version)?;
        Ok(Self {
            implementation,
            version,
        })
    }

    /// Adapter implementation identifier.
    pub fn implementation(&self) -> &str {
        &self.implementation
    }

    /// Adapter implementation version/profile.
    pub fn version(&self) -> &str {
        &self.version
    }
}

/// Raw adapter observation claiming one census is complete for one source root.
///
/// This remains untrusted until an actual [`ReviewCompletenessVerifier`]
/// qualifies it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewCompletenessObservation {
    version: ProtocolVersion,
    adapter: ReviewSourceAdapter,
    source_state: Digest,
    census_commitment: Digest,
    completeness_evidence: Digest,
}

impl ReviewCompletenessObservation {
    /// Construct a raw completeness observation.
    pub fn new(
        adapter: ReviewSourceAdapter,
        source_state: Digest,
        census_commitment: Digest,
        completeness_evidence: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            adapter,
            source_state,
            census_commitment,
            completeness_evidence,
        }
    }

    /// Adapter claiming completeness.
    pub fn adapter(&self) -> &ReviewSourceAdapter {
        &self.adapter
    }

    /// Exact collaboration/source-state root examined by the adapter.
    pub fn source_state(&self) -> &Digest {
        &self.source_state
    }

    /// Exact census the adapter claims is exhaustive.
    pub fn census_commitment(&self) -> &Digest {
        &self.census_commitment
    }

    /// Adapter-specific completeness evidence commitment.
    pub fn completeness_evidence(&self) -> &Digest {
        &self.completeness_evidence
    }

    /// Canonical v1 bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewCensusError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(COMPLETENESS_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_bytes(&mut out, "adapter_implementation", self.adapter.implementation.as_bytes())?;
        push_bytes(&mut out, "adapter_version", self.adapter.version.as_bytes())?;
        push_digest(&mut out, &self.source_state)?;
        push_digest(&mut out, &self.census_commitment)?;
        push_digest(&mut out, &self.completeness_evidence)?;
        Ok(out)
    }

    /// Exact raw observation commitment.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ReviewCensusError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ReviewCompletenessObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireObservation {
            version: ProtocolVersion,
            adapter: ReviewSourceAdapter,
            source_state: Digest,
            census_commitment: Digest,
            completeness_evidence: Digest,
        }

        let wire = WireObservation::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        ReviewSourceAdapter::new(
            wire.adapter.implementation,
            wire.adapter.version,
        )
        .map_err(D::Error::custom)
        .map(|adapter| Self::new(
            adapter,
            wire.source_state,
            wire.census_commitment,
            wire.completeness_evidence,
        ))
    }
}

/// Adapter-specific failure while independently verifying completeness.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum CompletenessVerifierError {
    /// Adapter could not establish that the supplied census was exhaustive.
    #[error("review source verifier rejected completeness")]
    Rejected,
}

/// External verifier for one concrete collaboration/review source.
///
/// Implementations should independently enumerate/verify the source state and
/// return an evidence commitment only when `census` is exhaustive for the
/// exact `observation.source_state`.
pub trait ReviewCompletenessVerifier {
    /// Stable identity of this verifier implementation/profile.
    fn adapter(&self) -> ReviewSourceAdapter;

    /// Verify that the census is complete and return verifier-side evidence.
    fn verify_complete(
        &self,
        census: &ReviewCensus,
        observation: &ReviewCompletenessObservation,
    ) -> Result<Digest, CompletenessVerifierError>;
}

/// Positive result that a named verifier accepted one exact census as complete.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCompleteReviewCensus {
    census: ReviewCensus,
    observation: ReviewCompletenessObservation,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

impl QualifiedCompleteReviewCensus {
    /// Canonical review census accepted as complete.
    pub fn census(&self) -> &ReviewCensus {
        &self.census
    }

    /// Raw adapter observation that was independently verified.
    pub fn observation(&self) -> &ReviewCompletenessObservation {
        &self.observation
    }

    /// Verifier-side evidence returned by the adapter implementation.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }

    /// Aggregate positive completeness-evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Verify and bind one adapter completeness claim to the exact census/source root.
pub fn qualify_complete_review_census<V: ReviewCompletenessVerifier>(
    census: ReviewCensus,
    observation: ReviewCompletenessObservation,
    verifier: &V,
) -> Result<QualifiedCompleteReviewCensus, ReviewCensusError> {
    if observation.adapter() != &verifier.adapter() {
        return Err(ReviewCensusError::AdapterMismatch);
    }
    if observation.source_state() != census.source_state() {
        return Err(ReviewCensusError::SourceStateMismatch);
    }

    let expected_census = census.digest(observation.census_commitment().algorithm())?;
    if observation.census_commitment() != &expected_census {
        return Err(ReviewCensusError::CensusCommitmentMismatch);
    }

    let verifier_evidence = verifier.verify_complete(&census, &observation)?;
    let algorithm = verifier_evidence.algorithm();
    let census_digest = census.digest(algorithm)?;
    let observation_digest = observation.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(COMPLETE_CENSUS_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, &census_digest)?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, &verifier_evidence)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(QualifiedCompleteReviewCensus {
        census,
        observation,
        verifier_evidence,
        evidence_commitment,
    })
}

/// Positive state proving a complete review census has no opposing review and
/// contains every review counted by one exact approval quorum.
///
/// This is still not merge authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ConflictFreeApprovalState {
    complete_census: QualifiedCompleteReviewCensus,
    approval_quorum_evidence: Digest,
    evidence_commitment: Digest,
}

impl ConflictFreeApprovalState {
    /// Complete census underlying this conflict-free conclusion.
    pub fn complete_census(&self) -> &QualifiedCompleteReviewCensus {
        &self.complete_census
    }

    /// Exact FORGE-007B approval-quorum evidence included in this result.
    pub fn approval_quorum_evidence(&self) -> &Digest {
        &self.approval_quorum_evidence
    }

    /// Aggregate conflict-free approval commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Combine an adapter-qualified complete census with one exact approval quorum.
pub fn qualify_conflict_free_approval_state(
    complete: QualifiedCompleteReviewCensus,
    quorum: &QualifiedApprovalQuorum,
) -> Result<ConflictFreeApprovalState, ReviewCensusError> {
    let census = complete.census();
    if census.project() != quorum.project() {
        return Err(ReviewCensusError::ProjectMismatch);
    }
    if census.proposal() != quorum.proposal() {
        return Err(ReviewCensusError::ProposalMismatch);
    }
    if census.authority_epoch() != quorum.authority_epoch() {
        return Err(ReviewCensusError::AuthorityEpochMismatch);
    }
    if census.repository_policy_state() != quorum.repository_policy_state() {
        return Err(ReviewCensusError::RepositoryPolicyMismatch);
    }

    if let Some(opposing) = census
        .entries()
        .iter()
        .find(|entry| entry.decision() == ReviewDecision::RequestChanges)
    {
        return Err(ReviewCensusError::OpposingReview(opposing.reviewer().clone()));
    }

    if quorum.reviewers().len() != quorum.review_evidence().len() {
        return Err(ReviewCensusError::MalformedApprovalQuorum);
    }

    let by_reviewer = census
        .entries()
        .iter()
        .map(|entry| (entry.reviewer().clone(), entry))
        .collect::<BTreeMap<_, _>>();

    for (reviewer, evidence) in quorum.reviewers().iter().zip(quorum.review_evidence()) {
        let entry = by_reviewer
            .get(reviewer)
            .ok_or_else(|| ReviewCensusError::QuorumReviewMissing(reviewer.clone()))?;
        if entry.decision() != ReviewDecision::Approve || entry.review_evidence() != evidence {
            return Err(ReviewCensusError::QuorumReviewMismatch(reviewer.clone()));
        }
    }

    let algorithm = quorum.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(CONFLICT_FREE_APPROVAL_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, complete.evidence_commitment())?;
    push_digest(&mut out, quorum.evidence_commitment())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ConflictFreeApprovalState {
        complete_census: complete,
        approval_quorum_evidence: quorum.evidence_commitment().clone(),
        evidence_commitment,
    })
}

/// Review-census construction/qualification failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewCensusError {
    /// Unsupported protocol version.
    #[error("unsupported Forge review-census protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Supplied review belongs to another project.
    #[error("review census project mismatch")]
    ProjectMismatch,
    /// Supplied review names another proposal.
    #[error("review census proposal mismatch")]
    ProposalMismatch,
    /// Supplied review names another authority epoch.
    #[error("review census authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Supplied review names another repository-policy state.
    #[error("review census repository-policy mismatch")]
    RepositoryPolicyMismatch,
    /// More than one security-significant review exists for a principal.
    #[error("ambiguous multiple security-significant reviews from {0}")]
    AmbiguousReviewer(PrincipalId),
    /// Review-set bound exceeded.
    #[error("too many reviews in census: {actual} > {max}")]
    TooManyReviews {
        /// Observed review count.
        actual: usize,
        /// Protocol-v1 maximum.
        max: usize,
    },
    /// Adapter field is empty or too long.
    #[error("invalid review-source adapter field {field}: length {len}")]
    InvalidAdapterField {
        /// Field name.
        field: &'static str,
        /// Observed byte length.
        len: usize,
    },
    /// Observation adapter identity does not match the verifier implementation.
    #[error("review completeness observation adapter does not match verifier")]
    AdapterMismatch,
    /// Observation source state does not match the census source root.
    #[error("review completeness source state does not match census")]
    SourceStateMismatch,
    /// Observation names another census commitment.
    #[error("review completeness observation names another census")]
    CensusCommitmentMismatch,
    /// Complete census contains an opposing RequestChanges review.
    #[error("complete review census contains RequestChanges by {0}")]
    OpposingReview(PrincipalId),
    /// Approval-quorum reviewer is absent from the complete census.
    #[error("approval-quorum reviewer missing from complete census: {0}")]
    QuorumReviewMissing(PrincipalId),
    /// Approval-quorum review evidence does not equal the census entry.
    #[error("approval-quorum evidence does not match complete census for {0}")]
    QuorumReviewMismatch(PrincipalId),
    /// Positive approval-quorum object has inconsistent reviewer/evidence vectors.
    #[error("approval-quorum reviewer/evidence cardinality mismatch")]
    MalformedApprovalQuorum,
    /// Canonical field exceeded encoding bounds.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed size.
        len: usize,
        /// Maximum size.
        max: usize,
    },
    /// Proposal canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// External completeness verifier rejected the census.
    #[error(transparent)]
    Completeness(#[from] CompletenessVerifierError),
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ReviewCensusError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewCensusError::UnsupportedProtocolVersion(version.get()))
    }
}

fn validate_adapter_field(field: &'static str, value: &str) -> Result<(), ReviewCensusError> {
    let len = value.as_bytes().len();
    if len == 0 || len > MAX_ADAPTER_FIELD_LEN {
        return Err(ReviewCensusError::InvalidAdapterField { field, len });
    }
    Ok(())
}

fn review_decision_code(decision: ReviewDecision) -> u8 {
    match decision {
        ReviewDecision::Approve => 1,
        ReviewDecision::RequestChanges => 2,
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewCensusError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewCensusError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewCensusError> {
    let len = u32::try_from(bytes.len()).map_err(|_| ReviewCensusError::CanonicalFieldTooLarge {
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
        AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant,
    };
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_proposal::ChangeProposal;
    use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
    use mycelix_forge_review::{
        bind_authenticated_review, qualify_review_authority, ReviewStatement,
    };
    use mycelix_forge_review_quorum::qualify_approval_quorum;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn epoch(project: ProjectIdentity, reviewers: &[PrincipalId]) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project,
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 100,
            valid_until_unix_ms: Some(10_000),
            grants: reviewers
                .iter()
                .cloned()
                .map(|reviewer| {
                    PrincipalGrant::new(reviewer, [Capability::ReviewSource]).unwrap()
                })
                .collect(),
            thresholds: vec![CapabilityRule::new(Capability::ReviewSource, 2).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn proposal(project: ProjectIdentity, epoch: &AuthorityEpoch) -> ChangeProposal {
        ChangeProposal::new(
            project,
            principal(0x29),
            epoch.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0x31),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
        )
        .unwrap()
    }

    fn review(
        proposal: &ChangeProposal,
        epoch: &AuthorityEpoch,
        reviewer: PrincipalId,
        decision: ReviewDecision,
        context_byte: u8,
    ) -> StructurallyAuthorizedReview {
        let proposal_id = proposal.proposal_id(DigestAlgorithm::Sha256).unwrap();
        let statement = ReviewStatement::new(
            proposal_id,
            reviewer.clone(),
            proposal.authority_epoch().clone(),
            proposal.repository_policy_state().clone(),
            decision,
            digest(context_byte),
        );
        let statement_digest = statement.digest(DigestAlgorithm::Sha256).unwrap();
        let binding = PrincipalBinding::new(
            digest(0x70),
            digest(context_byte.wrapping_add(1)),
            reviewer.clone(),
            digest(context_byte.wrapping_add(2)),
        );
        let request = PrincipalAuthenticationRequest::new(
            proposal.project().clone(),
            proposal.authority_epoch().clone(),
            reviewer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            statement_digest,
            [context_byte; 32],
        );
        let observation = AuthenticationObservation::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(context_byte.wrapping_add(3)),
            digest(context_byte.wrapping_add(4)),
        );
        let authentication =
            bind_principal_authentication(request, binding, epoch, observation).unwrap();
        let authenticated =
            bind_authenticated_review(proposal, statement, &authentication).unwrap();
        qualify_review_authority(authenticated, epoch, 1_000).unwrap()
    }

    struct ExactCensusVerifier {
        adapter: ReviewSourceAdapter,
        expected_census: Digest,
        evidence: Digest,
    }

    impl ReviewCompletenessVerifier for ExactCensusVerifier {
        fn adapter(&self) -> ReviewSourceAdapter {
            self.adapter.clone()
        }

        fn verify_complete(
            &self,
            census: &ReviewCensus,
            _observation: &ReviewCompletenessObservation,
        ) -> Result<Digest, CompletenessVerifierError> {
            let actual = census
                .digest(self.expected_census.algorithm())
                .map_err(|_| CompletenessVerifierError::Rejected)?;
            if actual != self.expected_census {
                return Err(CompletenessVerifierError::Rejected);
            }
            Ok(self.evidence.clone())
        }
    }

    fn qualify_complete(census: ReviewCensus) -> QualifiedCompleteReviewCensus {
        let adapter = ReviewSourceAdapter::new("test-review-log", "v1").unwrap();
        let census_digest = census.digest(DigestAlgorithm::Sha256).unwrap();
        let observation = ReviewCompletenessObservation::new(
            adapter.clone(),
            census.source_state().clone(),
            census_digest.clone(),
            digest(0xa1),
        );
        let verifier = ExactCensusVerifier {
            adapter,
            expected_census: census_digest,
            evidence: digest(0xa2),
        };
        qualify_complete_review_census(census, observation, &verifier).unwrap()
    }

    #[test]
    fn complete_two_approval_census_composes_with_quorum() {
        let project = project();
        let reviewers = vec![principal(0x20), principal(0x21), principal(0x22)];
        let epoch = epoch(project.clone(), &reviewers);
        let proposal = proposal(project, &epoch);
        let a = review(&proposal, &epoch, reviewers[0].clone(), ReviewDecision::Approve, 0x60);
        let b = review(&proposal, &epoch, reviewers[1].clone(), ReviewDecision::Approve, 0x61);
        let quorum = qualify_approval_quorum(&proposal, &epoch, 1_000, vec![a.clone(), b.clone()])
            .unwrap();
        let census = ReviewCensus::new(
            &proposal,
            digest(0x90),
            &[b, a],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let complete = qualify_complete(census);
        let state = qualify_conflict_free_approval_state(complete, &quorum).unwrap();
        assert_eq!(state.complete_census().census().entries().len(), 2);
    }

    #[test]
    fn census_order_is_canonical() {
        let project = project();
        let reviewers = vec![principal(0x20), principal(0x21)];
        let epoch = epoch(project.clone(), &reviewers);
        let proposal = proposal(project, &epoch);
        let a = review(&proposal, &epoch, reviewers[0].clone(), ReviewDecision::Approve, 0x60);
        let b = review(&proposal, &epoch, reviewers[1].clone(), ReviewDecision::Approve, 0x61);
        let ab = ReviewCensus::new(
            &proposal,
            digest(0x90),
            &[a.clone(), b.clone()],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let ba = ReviewCensus::new(
            &proposal,
            digest(0x90),
            &[b, a],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_eq!(
            ab.digest(DigestAlgorithm::Sha256).unwrap(),
            ba.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn same_reviewer_multiple_security_reviews_fail_closed() {
        let project = project();
        let reviewer = principal(0x20);
        let epoch = epoch(project.clone(), &[reviewer.clone(), principal(0x21)]);
        let proposal = proposal(project, &epoch);
        let approve = review(&proposal, &epoch, reviewer.clone(), ReviewDecision::Approve, 0x60);
        let reject = review(
            &proposal,
            &epoch,
            reviewer.clone(),
            ReviewDecision::RequestChanges,
            0x61,
        );
        assert_eq!(
            ReviewCensus::new(
                &proposal,
                digest(0x90),
                &[approve, reject],
                DigestAlgorithm::Sha256,
            )
            .unwrap_err(),
            ReviewCensusError::AmbiguousReviewer(reviewer)
        );
    }

    #[test]
    fn complete_census_with_request_changes_blocks_conflict_free_state() {
        let project = project();
        let reviewers = vec![principal(0x20), principal(0x21), principal(0x22)];
        let epoch = epoch(project.clone(), &reviewers);
        let proposal = proposal(project, &epoch);
        let a = review(&proposal, &epoch, reviewers[0].clone(), ReviewDecision::Approve, 0x60);
        let b = review(&proposal, &epoch, reviewers[1].clone(), ReviewDecision::Approve, 0x61);
        let opposing = review(
            &proposal,
            &epoch,
            reviewers[2].clone(),
            ReviewDecision::RequestChanges,
            0x62,
        );
        let quorum = qualify_approval_quorum(&proposal, &epoch, 1_000, vec![a.clone(), b.clone()])
            .unwrap();
        let census = ReviewCensus::new(
            &proposal,
            digest(0x90),
            &[a, b, opposing],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let complete = qualify_complete(census);
        assert_eq!(
            qualify_conflict_free_approval_state(complete, &quorum).unwrap_err(),
            ReviewCensusError::OpposingReview(reviewers[2].clone())
        );
    }

    #[test]
    fn completeness_verifier_detects_omitted_review() {
        let project = project();
        let reviewers = vec![principal(0x20), principal(0x21)];
        let epoch = epoch(project.clone(), &reviewers);
        let proposal = proposal(project, &epoch);
        let a = review(&proposal, &epoch, reviewers[0].clone(), ReviewDecision::Approve, 0x60);
        let b = review(&proposal, &epoch, reviewers[1].clone(), ReviewDecision::Approve, 0x61);
        let full = ReviewCensus::new(
            &proposal,
            digest(0x90),
            &[a.clone(), b],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let subset = ReviewCensus::new(
            &proposal,
            digest(0x90),
            &[a],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let adapter = ReviewSourceAdapter::new("test-review-log", "v1").unwrap();
        let observation = ReviewCompletenessObservation::new(
            adapter.clone(),
            subset.source_state().clone(),
            subset.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0xa1),
        );
        let verifier = ExactCensusVerifier {
            adapter,
            expected_census: full.digest(DigestAlgorithm::Sha256).unwrap(),
            evidence: digest(0xa2),
        };
        assert_eq!(
            qualify_complete_review_census(subset, observation, &verifier).unwrap_err(),
            ReviewCensusError::Completeness(CompletenessVerifierError::Rejected)
        );
    }
}

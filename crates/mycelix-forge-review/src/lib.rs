// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable review attestations for Mycelix Forge.
//!
//! Review authority is deliberately decomposed:
//!
//! 1. [`ReviewStatement`] is the immutable decision over one exact proposal;
//! 2. [`EvidenceBoundReviewAttestation`] proves that FORGE-005A evidence names
//!    this exact reviewer and review statement;
//! 3. [`StructurallyEligibleReview`] proves the supplied authority epoch says
//!    that reviewer is eligible for `ReviewSource` at a caller-supplied time.
//!
//! None of these types alone proves provider cryptography, trusted time,
//! review quorum, merge authorization, or repository correctness.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication::EvidenceBoundPrincipalAuthentication;
use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const REVIEW_STATEMENT_DOMAIN_V1: &[u8] = b"mycelix-forge/review-statement/v1\0";
const EVIDENCE_BOUND_REVIEW_DOMAIN_V1: &[u8] = b"mycelix-forge/evidence-bound-review/v1\0";
const STRUCTURAL_ELIGIBILITY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/structurally-eligible-review/v1\0";

/// Security-significant review decisions.
///
/// Ordinary discussion/comments are intentionally not review authority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewDecision {
    /// Reviewer approves this exact immutable proposal subject.
    Approve,
    /// Reviewer requests changes to this exact immutable proposal subject.
    RequestChanges,
}

impl ReviewDecision {
    const fn code(self) -> u8 {
        match self {
            Self::Approve => 1,
            Self::RequestChanges => 2,
        }
    }
}

/// Stable identifier for one exact review statement.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ReviewStatementId(Digest);

impl ReviewStatementId {
    /// Construct from an already-derived statement commitment.
    pub fn new(commitment: Digest) -> Self {
        Self(commitment)
    }

    /// Algorithm-qualified statement commitment.
    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

/// Immutable review action over one exact `ChangeProposalId`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewStatement {
    version: ProtocolVersion,
    proposal: ChangeProposalId,
    reviewer: PrincipalId,
    decision: ReviewDecision,
    review_context: Digest,
}

impl ReviewStatement {
    /// Construct a v1 review statement directly from the exact proposal.
    pub fn new(
        proposal: &ChangeProposal,
        reviewer: PrincipalId,
        decision: ReviewDecision,
        review_context: Digest,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ReviewError> {
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            proposal: proposal.proposal_id(commitment_algorithm)?,
            reviewer,
            decision,
            review_context,
        })
    }

    fn from_parts(
        proposal: ChangeProposalId,
        reviewer: PrincipalId,
        decision: ReviewDecision,
        review_context: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            proposal,
            reviewer,
            decision,
            review_context,
        }
    }

    /// Proposal being reviewed.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Reviewer claiming this decision.
    pub fn reviewer(&self) -> &PrincipalId {
        &self.reviewer
    }

    /// Security-significant decision.
    pub const fn decision(&self) -> ReviewDecision {
        self.decision
    }

    /// Immutable review rationale/checklist/result commitment.
    pub fn review_context(&self) -> &Digest {
        &self.review_context
    }

    /// Canonical v1 statement bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(REVIEW_STATEMENT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, self.reviewer.commitment())?;
        out.push(self.decision.code());
        push_digest(&mut out, &self.review_context)?;
        Ok(out)
    }

    /// Stable identifier for this exact review statement.
    pub fn statement_id(&self, algorithm: DigestAlgorithm) -> Result<ReviewStatementId, ReviewError> {
        Ok(ReviewStatementId::new(Digest::of_bytes(
            algorithm,
            &self.canonical_bytes()?,
        )))
    }
}

impl<'de> Deserialize<'de> for ReviewStatement {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireStatement {
            version: ProtocolVersion,
            proposal: ChangeProposalId,
            reviewer: PrincipalId,
            decision: ReviewDecision,
            review_context: Digest,
        }

        let wire = WireStatement::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Ok(Self::from_parts(
            wire.proposal,
            wire.reviewer,
            wire.decision,
            wire.review_context,
        ))
    }
}

/// Positive structural result binding FORGE-005A evidence to one exact review.
///
/// This deliberately says **evidence-bound**, not "signed" or
/// "cryptographically authenticated": FORGE-005A provider evidence is opaque
/// until a concrete provider verifier establishes it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct EvidenceBoundReviewAttestation {
    project: ProjectIdentity,
    authority_epoch: Digest,
    statement: ReviewStatement,
    authentication_evidence: Digest,
    evidence_commitment: Digest,
}

impl EvidenceBoundReviewAttestation {
    /// Project containing the reviewed proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact authority epoch inherited from the proposal/authentication request.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact evidence-bound review statement.
    pub fn statement(&self) -> &ReviewStatement {
        &self.statement
    }

    /// FORGE-005A evidence commitment bound to this statement.
    pub fn authentication_evidence(&self) -> &Digest {
        &self.authentication_evidence
    }

    /// Aggregate evidence-bound review commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind FORGE-005A principal-authentication evidence to one exact review.
pub fn bind_review_evidence(
    proposal: &ChangeProposal,
    statement: ReviewStatement,
    authentication: &EvidenceBoundPrincipalAuthentication,
    authority_epoch: &AuthorityEpoch,
) -> Result<EvidenceBoundReviewAttestation, ReviewError> {
    let expected_proposal = proposal.proposal_id(statement.proposal().commitment().algorithm())?;
    if statement.proposal() != &expected_proposal {
        return Err(ReviewError::ProposalMismatch);
    }

    if authority_epoch.project() != proposal.project() {
        return Err(ReviewError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch {
        return Err(ReviewError::AuthorityEpochMismatch);
    }

    let request = authentication.request();
    if request.project() != proposal.project() {
        return Err(ReviewError::ProjectMismatch);
    }
    if request.principal() != statement.reviewer() {
        return Err(ReviewError::ReviewerMismatch);
    }
    if request.capability() != Capability::ReviewSource {
        return Err(ReviewError::WrongAuthenticationCapability);
    }
    if request.authority_epoch() != &expected_epoch {
        return Err(ReviewError::AuthenticationAuthorityMismatch);
    }

    let expected_statement = statement.statement_id(request.action_subject().algorithm())?;
    if request.action_subject() != expected_statement.commitment() {
        return Err(ReviewError::AuthenticationSubjectMismatch);
    }

    let algorithm = authentication.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(EVIDENCE_BOUND_REVIEW_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, &expected_epoch)?;
    push_digest(&mut out, statement.statement_id(algorithm)?.commitment())?;
    push_digest(&mut out, authentication.evidence_commitment())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(EvidenceBoundReviewAttestation {
        project: proposal.project().clone(),
        authority_epoch: expected_epoch,
        statement,
        authentication_evidence: authentication.evidence_commitment().clone(),
        evidence_commitment,
    })
}

/// Positive structural result that the named reviewer is eligible for
/// `ReviewSource` at one caller-supplied observation time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructurallyEligibleReview {
    review: EvidenceBoundReviewAttestation,
    observed_at_unix_ms: u64,
    evidence_commitment: Digest,
}

impl StructurallyEligibleReview {
    /// Evidence-bound review whose reviewer was structurally eligible.
    pub fn review(&self) -> &EvidenceBoundReviewAttestation {
        &self.review
    }

    /// Caller-supplied observation time used for authority evaluation.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Aggregate structural-eligibility commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate `ReviewSource` eligibility in the exact authority epoch.
pub fn qualify_review_eligibility(
    review: EvidenceBoundReviewAttestation,
    authority_epoch: &AuthorityEpoch,
    observed_at_unix_ms: u64,
) -> Result<StructurallyEligibleReview, ReviewError> {
    if authority_epoch.project() != review.project() {
        return Err(ReviewError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(review.authority_epoch().algorithm())?;
    if review.authority_epoch() != &expected_epoch {
        return Err(ReviewError::AuthorityEpochMismatch);
    }
    if !authority_epoch.is_principal_eligible(
        review.statement().reviewer(),
        Capability::ReviewSource,
        observed_at_unix_ms,
    ) {
        return Err(ReviewError::ReviewerNotEligible);
    }

    let algorithm = review.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(STRUCTURAL_ELIGIBILITY_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, review.evidence_commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    out.extend_from_slice(&observed_at_unix_ms.to_be_bytes());
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(StructurallyEligibleReview {
        review,
        observed_at_unix_ms,
        evidence_commitment,
    })
}

/// Review-attestation validation failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewError {
    /// Unsupported review protocol version.
    #[error("unsupported Forge review protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Statement names a different proposal.
    #[error("review statement does not name the supplied proposal")]
    ProposalMismatch,
    /// Supplied authority epoch does not match the proposal/review context.
    #[error("review authority epoch does not match proposal")]
    AuthorityEpochMismatch,
    /// Authentication is for another project.
    #[error("review authentication project does not match proposal")]
    ProjectMismatch,
    /// Authentication principal differs from the reviewer.
    #[error("review authentication principal does not match reviewer")]
    ReviewerMismatch,
    /// Authentication request is not scoped to `ReviewSource`.
    #[error("review authentication is not scoped to ReviewSource")]
    WrongAuthenticationCapability,
    /// Authentication names another authority epoch.
    #[error("review authentication authority epoch does not match proposal")]
    AuthenticationAuthorityMismatch,
    /// Authentication is over a different review action subject.
    #[error("review authentication action subject does not match review statement")]
    AuthenticationSubjectMismatch,
    /// Reviewer is not structurally eligible at the supplied time.
    #[error("reviewer is not eligible for ReviewSource in the supplied authority epoch/time")]
    ReviewerNotEligible,
    /// Canonical field exceeded its encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed size.
        len: usize,
        /// Maximum size.
        max: usize,
    },
    /// Proposal canonicalization error.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Authority canonicalization error.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ReviewError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_project_identity(out: &mut Vec<u8>, project: &ProjectIdentity) -> Result<(), ReviewError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(out: &mut Vec<u8>, field: &'static str, bytes: &[u8]) -> Result<(), ReviewError> {
    let len = u32::try_from(bytes.len()).map_err(|_| ReviewError::CanonicalFieldTooLarge {
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
        AuthenticationObservation, PrincipalAuthenticationRequest, PrincipalBinding,
        bind_principal_authentication,
    };
    use mycelix_forge_authority::{AuthorityEpochParts, CapabilityRule, PrincipalGrant};
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };

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

    fn authority(reviewer: PrincipalId, valid_until: Option<u64>) -> AuthorityEpoch {
        let manager = principal(0x10);
        AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: valid_until,
            grants: vec![
                PrincipalGrant::new(manager, [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(reviewer, [Capability::ReviewSource]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn project_policy() -> ProjectPolicyStateV1 {
        let project = project();
        let trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![TrustedProviderVerifierV1::new(digest(0x90), digest(0x91))],
        )
        .unwrap();
        ProjectPolicyStateV1::new(
            project,
            0,
            None,
            &trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn proposal(authority: &AuthorityEpoch) -> ChangeProposal {
        let repository_policy =
            RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        let project_policy = project_policy();
        ChangeProposal::new(
            principal(0x30),
            authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn binding(reviewer: &PrincipalId) -> PrincipalBinding {
        PrincipalBinding::new(digest(0x70), digest(0x71), reviewer.clone(), digest(0x72))
    }

    fn evidence_for(
        authority: &AuthorityEpoch,
        statement: &ReviewStatement,
        reviewer: PrincipalId,
        capability: Capability,
        action_subject: Digest,
    ) -> EvidenceBoundPrincipalAuthentication {
        let binding = binding(&reviewer);
        let request = PrincipalAuthenticationRequest::new(
            project(),
            authority.digest(DigestAlgorithm::Sha256).unwrap(),
            reviewer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            capability,
            action_subject,
            [0x80; 32],
        );
        let observation = AuthenticationObservation::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0x81),
            digest(0x82),
        );
        let _ = statement;
        bind_principal_authentication(request, binding, authority, observation).unwrap()
    }

    #[test]
    fn decision_and_context_are_part_of_the_exact_action_subject() {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone(), None);
        let proposal = proposal(&authority);
        let approve = ReviewStatement::new(
            &proposal,
            reviewer.clone(),
            ReviewDecision::Approve,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let changes = ReviewStatement::new(
            &proposal,
            reviewer.clone(),
            ReviewDecision::RequestChanges,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let context_changed = ReviewStatement::new(
            &proposal,
            reviewer,
            ReviewDecision::Approve,
            digest(0x61),
            DigestAlgorithm::Sha256,
        )
        .unwrap();

        assert_ne!(approve.statement_id(DigestAlgorithm::Sha256).unwrap(), changes.statement_id(DigestAlgorithm::Sha256).unwrap());
        assert_ne!(approve.statement_id(DigestAlgorithm::Sha256).unwrap(), context_changed.statement_id(DigestAlgorithm::Sha256).unwrap());
    }

    #[test]
    fn fake_provider_commitments_can_be_evidence_bound_but_are_not_provider_crypto_proof() {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone(), None);
        let proposal = proposal(&authority);
        let statement = ReviewStatement::new(&proposal, reviewer.clone(), ReviewDecision::Approve, digest(0x60), DigestAlgorithm::Sha256).unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let authentication = evidence_for(&authority, &statement, reviewer, Capability::ReviewSource, statement_id.commitment().clone());
        assert!(bind_review_evidence(&proposal, statement, &authentication, &authority).is_ok());
    }

    #[test]
    fn authentication_must_sign_the_exact_review_statement() {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone(), None);
        let proposal = proposal(&authority);
        let statement = ReviewStatement::new(&proposal, reviewer.clone(), ReviewDecision::Approve, digest(0x60), DigestAlgorithm::Sha256).unwrap();
        let authentication = evidence_for(&authority, &statement, reviewer, Capability::ReviewSource, digest(0x99));
        assert_eq!(bind_review_evidence(&proposal, statement, &authentication, &authority).unwrap_err(), ReviewError::AuthenticationSubjectMismatch);
    }

    #[test]
    fn review_source_capability_and_reviewer_identity_are_exact() {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone(), None);
        let proposal = proposal(&authority);
        let statement = ReviewStatement::new(&proposal, reviewer.clone(), ReviewDecision::Approve, digest(0x60), DigestAlgorithm::Sha256).unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let wrong_cap = evidence_for(&authority, &statement, reviewer.clone(), Capability::ManageAuthority, statement_id.commitment().clone());
        assert_eq!(bind_review_evidence(&proposal, statement.clone(), &wrong_cap, &authority).unwrap_err(), ReviewError::WrongAuthenticationCapability);
        let other = principal(0x29);
        let wrong_reviewer = evidence_for(&authority, &statement, other, Capability::ReviewSource, statement_id.commitment().clone());
        assert_eq!(bind_review_evidence(&proposal, statement, &wrong_reviewer, &authority).unwrap_err(), ReviewError::ReviewerMismatch);
    }

    #[test]
    fn statement_for_old_proposal_does_not_apply_after_proposal_mutation() {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone(), None);
        let original = proposal(&authority);
        let statement = ReviewStatement::new(&original, reviewer.clone(), ReviewDecision::Approve, digest(0x60), DigestAlgorithm::Sha256).unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let authentication = evidence_for(&authority, &statement, reviewer, Capability::ReviewSource, statement_id.commitment().clone());
        let repository_policy = RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        let project_policy = project_policy();
        let mutated = ChangeProposal::new(
            principal(0x30), &authority, &project_policy, &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(), git(0x40), git(0x43), git(0x42),
            digest(0x50), vec![], DigestAlgorithm::Sha256,
        ).unwrap();
        assert_eq!(bind_review_evidence(&mutated, statement, &authentication, &authority).unwrap_err(), ReviewError::ProposalMismatch);
    }

    #[test]
    fn structural_eligibility_is_separate_from_evidence_binding_and_time_is_external() {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone(), Some(2_000));
        let proposal = proposal(&authority);
        let statement = ReviewStatement::new(&proposal, reviewer.clone(), ReviewDecision::Approve, digest(0x60), DigestAlgorithm::Sha256).unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let authentication = evidence_for(&authority, &statement, reviewer, Capability::ReviewSource, statement_id.commitment().clone());
        let bound = bind_review_evidence(&proposal, statement, &authentication, &authority).unwrap();
        assert!(qualify_review_eligibility(bound.clone(), &authority, 1_500).is_ok());
        assert_eq!(qualify_review_eligibility(bound, &authority, 2_000).unwrap_err(), ReviewError::ReviewerNotEligible);
    }

    #[test]
    fn structurally_authenticated_outsider_is_not_review_eligible() {
        let eligible = principal(0x20);
        let outsider = principal(0x2a);
        let authority = authority(eligible, None);
        let proposal = proposal(&authority);
        let statement = ReviewStatement::new(&proposal, outsider.clone(), ReviewDecision::Approve, digest(0x60), DigestAlgorithm::Sha256).unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let authentication = evidence_for(&authority, &statement, outsider, Capability::ReviewSource, statement_id.commitment().clone());
        let bound = bind_review_evidence(&proposal, statement, &authentication, &authority).unwrap();
        assert_eq!(qualify_review_eligibility(bound, &authority, 1_500).unwrap_err(), ReviewError::ReviewerNotEligible);
    }

    #[test]
    fn statement_serde_round_trip_preserves_identity() {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone(), None);
        let proposal = proposal(&authority);
        let statement = ReviewStatement::new(&proposal, reviewer, ReviewDecision::RequestChanges, digest(0x60), DigestAlgorithm::Sha256).unwrap();
        let json = serde_json::to_vec(&statement).unwrap();
        let decoded: ReviewStatement = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, statement);
        assert_eq!(decoded.statement_id(DigestAlgorithm::Sha256).unwrap(), statement.statement_id(DigestAlgorithm::Sha256).unwrap());
    }
}

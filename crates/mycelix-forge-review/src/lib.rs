// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable authenticated review attestations for Mycelix Forge.
//!
//! Reviews are split into three layers:
//!
//! 1. [`ReviewStatement`] — immutable reviewer decision over one exact proposal;
//! 2. [`AuthenticatedReviewAttestation`] — FORGE-005A proves the named reviewer
//!    authenticated that exact statement subject;
//! 3. [`StructurallyAuthorizedReview`] — the exact authority epoch says the
//!    reviewer is eligible for `ReviewSource` at an externally supplied
//!    observation time.
//!
//! This crate does not establish trusted time, threshold/quorum satisfaction,
//! merge authorization, or repository correctness.

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
const AUTHENTICATED_REVIEW_DOMAIN_V1: &[u8] = b"mycelix-forge/authenticated-review/v1\0";
const STRUCTURAL_REVIEW_AUTHORITY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/structurally-authorized-review/v1\0";

/// Security-significant review decisions.
///
/// Ordinary comments/discussion are intentionally outside this enum and do
/// not participate in review authority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewDecision {
    /// Reviewer approves this exact proposal subject.
    Approve,
    /// Reviewer explicitly requests changes to this exact proposal subject.
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

/// Immutable statement a reviewer authenticates.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewStatement {
    version: ProtocolVersion,
    proposal: ChangeProposalId,
    reviewer: PrincipalId,
    authority_epoch: Digest,
    repository_policy_state: Digest,
    decision: ReviewDecision,
    review_context: Digest,
}

impl ReviewStatement {
    /// Construct a v1 review statement.
    pub fn new(
        proposal: ChangeProposalId,
        reviewer: PrincipalId,
        authority_epoch: Digest,
        repository_policy_state: Digest,
        decision: ReviewDecision,
        review_context: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            proposal,
            reviewer,
            authority_epoch,
            repository_policy_state,
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

    /// Exact authority epoch under which this review is evaluated.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact repository-policy state inherited from the proposal.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
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
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.repository_policy_state)?;
        out.push(self.decision.code());
        push_digest(&mut out, &self.review_context)?;
        Ok(out)
    }

    /// Identifier for this exact review statement.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ReviewError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
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
            authority_epoch: Digest,
            repository_policy_state: Digest,
            decision: ReviewDecision,
            review_context: Digest,
        }

        let wire = WireStatement::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Ok(Self::new(
            wire.proposal,
            wire.reviewer,
            wire.authority_epoch,
            wire.repository_policy_state,
            wire.decision,
            wire.review_context,
        ))
    }
}

/// Positive result binding one authenticated Forge principal to one exact
/// review statement and proposal.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AuthenticatedReviewAttestation {
    project: ProjectIdentity,
    statement: ReviewStatement,
    authentication_evidence: Digest,
    evidence_commitment: Digest,
}

impl AuthenticatedReviewAttestation {
    /// Project containing the reviewed proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact authenticated review statement.
    pub fn statement(&self) -> &ReviewStatement {
        &self.statement
    }

    /// FORGE-005A evidence commitment authenticating the reviewer/statement.
    pub fn authentication_evidence(&self) -> &Digest {
        &self.authentication_evidence
    }

    /// Aggregate authenticated-review commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind FORGE-005A principal authentication to one exact review statement.
///
/// Forge protocol v1 deliberately requires the review's authority epoch and
/// repository-policy state to equal the proposal's own exact context. If
/// either changes, the proposal/review must be reissued rather than carrying
/// an old approval across an authority/policy transition.
pub fn bind_authenticated_review(
    proposal: &ChangeProposal,
    statement: ReviewStatement,
    authentication: &EvidenceBoundPrincipalAuthentication,
) -> Result<AuthenticatedReviewAttestation, ReviewError> {
    let expected_proposal = proposal.proposal_id(statement.proposal().commitment().algorithm())?;
    if statement.proposal() != &expected_proposal {
        return Err(ReviewError::ProposalMismatch);
    }
    if statement.authority_epoch() != proposal.authority_epoch() {
        return Err(ReviewError::AuthorityEpochMismatch);
    }
    if statement.repository_policy_state() != proposal.repository_policy_state() {
        return Err(ReviewError::RepositoryPolicyMismatch);
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
    if request.authority_epoch() != statement.authority_epoch() {
        return Err(ReviewError::AuthenticationAuthorityMismatch);
    }

    let expected_statement = statement.digest(request.action_subject().algorithm())?;
    if request.action_subject() != &expected_statement {
        return Err(ReviewError::AuthenticationSubjectMismatch);
    }

    let algorithm = authentication.evidence_commitment().algorithm();
    let evidence_commitment = authenticated_review_commitment(
        algorithm,
        proposal.project(),
        &statement,
        authentication.evidence_commitment(),
    )?;

    Ok(AuthenticatedReviewAttestation {
        project: proposal.project().clone(),
        statement,
        authentication_evidence: authentication.evidence_commitment().clone(),
        evidence_commitment,
    })
}

/// Positive structural authority result for one authenticated review.
///
/// `observed_at_unix_ms` is supplied by the caller. This type proves authority
/// eligibility *conditional on that observation time*; it does not prove a
/// trusted clock or timestamp authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructurallyAuthorizedReview {
    authenticated_review: AuthenticatedReviewAttestation,
    observed_at_unix_ms: u64,
    evidence_commitment: Digest,
}

impl StructurallyAuthorizedReview {
    /// Authenticated review that was structurally authorized.
    pub fn authenticated_review(&self) -> &AuthenticatedReviewAttestation {
        &self.authenticated_review
    }

    /// External observation time used for the authority validity check.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Aggregate structural-authorization evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Check that the exact authority epoch structurally permits this reviewer to
/// exercise `ReviewSource` at `observed_at_unix_ms`.
///
/// This does not count review quorum or establish trusted time.
pub fn qualify_review_authority(
    authenticated: AuthenticatedReviewAttestation,
    authority_epoch: &AuthorityEpoch,
    observed_at_unix_ms: u64,
) -> Result<StructurallyAuthorizedReview, ReviewError> {
    if authority_epoch.project() != authenticated.project() {
        return Err(ReviewError::ProjectMismatch);
    }

    let expected_epoch = authority_epoch.digest(
        authenticated
            .statement()
            .authority_epoch()
            .algorithm(),
    )?;
    if authenticated.statement().authority_epoch() != &expected_epoch {
        return Err(ReviewError::AuthorityEpochMismatch);
    }

    if !authority_epoch.is_principal_eligible(
        authenticated.statement().reviewer(),
        Capability::ReviewSource,
        observed_at_unix_ms,
    ) {
        return Err(ReviewError::ReviewerNotEligible);
    }

    let algorithm = authenticated.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(STRUCTURAL_REVIEW_AUTHORITY_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, authenticated.evidence_commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    out.extend_from_slice(&observed_at_unix_ms.to_be_bytes());
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(StructurallyAuthorizedReview {
        authenticated_review: authenticated,
        observed_at_unix_ms,
        evidence_commitment,
    })
}

fn authenticated_review_commitment(
    algorithm: DigestAlgorithm,
    project: &ProjectIdentity,
    statement: &ReviewStatement,
    authentication_evidence: &Digest,
) -> Result<Digest, ReviewError> {
    let mut out = Vec::new();
    out.extend_from_slice(AUTHENTICATED_REVIEW_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, project)?;
    push_digest(&mut out, &statement.digest(algorithm)?)?;
    push_digest(&mut out, authentication_evidence)?;
    Ok(Digest::of_bytes(algorithm, &out))
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
    /// Proposal/review/authentication authority epoch differs.
    #[error("review authority epoch does not match")]
    AuthorityEpochMismatch,
    /// Proposal/review repository-policy state differs.
    #[error("review repository-policy state does not match proposal")]
    RepositoryPolicyMismatch,
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
    #[error("review authentication authority epoch does not match statement")]
    AuthenticationAuthorityMismatch,
    /// Authentication is over a different action subject.
    #[error("review authentication action subject does not match statement")]
    AuthenticationSubjectMismatch,
    /// Reviewer is not structurally eligible for `ReviewSource` at the supplied time.
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
        bind_principal_authentication, AuthenticationObservation, PrincipalAuthenticationRequest,
        PrincipalBinding,
    };
    use mycelix_forge_authority::{
        AuthorityEpochParts, CapabilityRule, PrincipalGrant,
    };
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};

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

    fn reviewer() -> PrincipalId {
        PrincipalId::new(digest(0x20))
    }

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn epoch(project: ProjectIdentity, principal: PrincipalId) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project,
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 100,
            valid_until_unix_ms: Some(10_000),
            grants: vec![PrincipalGrant::new(
                principal,
                [Capability::ReviewSource, Capability::ManageAuthority],
            )
            .unwrap()],
            thresholds: vec![
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn fixture() -> (
        ChangeProposal,
        ReviewStatement,
        EvidenceBoundPrincipalAuthentication,
        AuthorityEpoch,
    ) {
        let project = project();
        let reviewer = reviewer();
        let epoch = epoch(project.clone(), reviewer.clone());
        let epoch_digest = epoch.digest(DigestAlgorithm::Sha256).unwrap();
        let proposal = ChangeProposal::new(
            project.clone(),
            PrincipalId::new(digest(0x21)),
            epoch_digest.clone(),
            digest(0x31),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
        )
        .unwrap();
        let proposal_id = proposal.proposal_id(DigestAlgorithm::Sha256).unwrap();
        let statement = ReviewStatement::new(
            proposal_id,
            reviewer.clone(),
            epoch_digest.clone(),
            proposal.repository_policy_state().clone(),
            ReviewDecision::Approve,
            digest(0x60),
        );
        let statement_digest = statement.digest(DigestAlgorithm::Sha256).unwrap();
        let binding = PrincipalBinding::new(
            digest(0x70),
            digest(0x71),
            reviewer.clone(),
            digest(0x72),
        );
        let binding_digest = binding.digest(DigestAlgorithm::Sha256).unwrap();
        let request = PrincipalAuthenticationRequest::new(
            project,
            epoch_digest,
            reviewer,
            binding_digest,
            Capability::ReviewSource,
            statement_digest,
            [0x80; 32],
        );
        let request_digest = request.digest(DigestAlgorithm::Sha256).unwrap();
        let observation = AuthenticationObservation::new(
            request_digest,
            digest(0x81),
            digest(0x82),
        );
        let authentication =
            bind_principal_authentication(request, binding, &epoch, observation).unwrap();
        (proposal, statement, authentication, epoch)
    }

    #[test]
    fn exact_review_authenticates_and_structurally_authorizes() {
        let (proposal, statement, authentication, epoch) = fixture();
        let authenticated = bind_authenticated_review(&proposal, statement, &authentication).unwrap();
        let authorized = qualify_review_authority(authenticated, &epoch, 1_000).unwrap();
        assert_eq!(authorized.authenticated_review().statement().decision(), ReviewDecision::Approve);
    }

    #[test]
    fn proposal_mutation_invalidates_old_review() {
        let (proposal, statement, authentication, _) = fixture();
        let mutated = ChangeProposal::new(
            proposal.project().clone(),
            proposal.proposer().clone(),
            proposal.authority_epoch().clone(),
            proposal.repository_policy_state().clone(),
            proposal.target_ref().clone(),
            proposal.base_revision().clone(),
            git(0x49),
            proposal.resulting_tree().clone(),
            proposal.change_intent().clone(),
            proposal.dependencies().to_vec(),
        )
        .unwrap();
        assert_eq!(
            bind_authenticated_review(&mutated, statement, &authentication).unwrap_err(),
            ReviewError::ProposalMismatch
        );
    }

    #[test]
    fn wrong_authentication_subject_fails() {
        let (proposal, statement, _, epoch) = fixture();
        let reviewer = statement.reviewer().clone();
        let binding = PrincipalBinding::new(digest(0x70), digest(0x71), reviewer.clone(), digest(0x72));
        let request = PrincipalAuthenticationRequest::new(
            proposal.project().clone(),
            proposal.authority_epoch().clone(),
            reviewer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            digest(0x99),
            [0x80; 32],
        );
        let observation = AuthenticationObservation::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0x81),
            digest(0x82),
        );
        let auth = bind_principal_authentication(request, binding, &epoch, observation).unwrap();
        assert_eq!(
            bind_authenticated_review(&proposal, statement, &auth).unwrap_err(),
            ReviewError::AuthenticationSubjectMismatch
        );
    }

    #[test]
    fn authentication_does_not_bypass_authority_validity() {
        let (proposal, statement, authentication, epoch) = fixture();
        let authenticated = bind_authenticated_review(&proposal, statement, &authentication).unwrap();
        assert_eq!(
            qualify_review_authority(authenticated, &epoch, 20_000).unwrap_err(),
            ReviewError::ReviewerNotEligible
        );
    }

    #[test]
    fn statement_round_trip_preserves_digest() {
        let (_, statement, _, _) = fixture();
        let json = serde_json::to_vec(&statement).unwrap();
        let decoded: ReviewStatement = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, statement);
        assert_eq!(
            decoded.digest(DigestAlgorithm::Sha256).unwrap(),
            statement.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Project-policy authority adoption of one exact proposal review policy.
//!
//! Review-policy adoption is governed by [`Capability::ManagePolicy`], not
//! `ManageAuthority`. The latter governs the authority structure itself; using
//! it for ordinary policy adoption would conflate two deliberately separate
//! capabilities in the Forge authority kernel.
//!
//! ```text
//! exact review policy + proposal review-policy context
//! + exact adoption statement
//! + project-policy-trusted Xenia authentication(ManagePolicy)
//! + ManagePolicy eligibility
//! + distinct ManagePolicy threshold
//!     -> ProposalReviewPolicyAuthorityQuorumV1
//! ```
//!
//! The positive result remains structural in time and proposal-specific. It is
//! not a trusted-current policy theorem and is not merge authorization.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication_policy::ProjectPolicyTrustedXeniaAuthenticationV1;
use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_acceptance_policy::{
    ProposalReviewPolicyContextV1, ReviewAcceptancePolicyError, ReviewAcceptancePolicyV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const ADOPTION_STATEMENT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/proposal-review-policy-adoption-statement/v1\0";
const POLICY_ATTESTATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/proposal-review-policy-authority-attestation/v1\0";
const POLICY_QUORUM_DOMAIN_V1: &[u8] =
    b"mycelix-forge/proposal-review-policy-authority-quorum/v1\0";

/// Stable identity for one exact proposal review-policy adoption statement.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ProposalReviewPolicyAdoptionStatementId(Digest);

impl ProposalReviewPolicyAdoptionStatementId {
    /// Construct from an already-derived commitment.
    pub fn new(commitment: Digest) -> Self {
        Self(commitment)
    }

    /// Algorithm-qualified statement commitment.
    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

/// Exact statement asking project policy authority to adopt one exact review
/// policy for one exact proposal.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProposalReviewPolicyAdoptionStatementV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    review_policy: Digest,
    review_policy_context: Digest,
    adoption_context: Digest,
}

impl ProposalReviewPolicyAdoptionStatementV1 {
    /// Construct one exact proposal review-policy adoption statement.
    pub fn new(
        proposal: &ChangeProposal,
        policy: &ReviewAcceptancePolicyV1,
        policy_context: &ProposalReviewPolicyContextV1,
        adoption_context: Digest,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ReviewPolicyAdoptionError> {
        validate_policy_context(proposal, policy, policy_context, commitment_algorithm)?;
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project: proposal.project().clone(),
            proposal: proposal.proposal_id(commitment_algorithm)?,
            review_policy: policy.digest(commitment_algorithm)?,
            review_policy_context: policy_context.digest(commitment_algorithm)?,
            adoption_context,
        })
    }

    /// Project whose policy authority is asked to adopt the review policy.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal governed by this review policy.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact review-policy commitment.
    pub fn review_policy(&self) -> &Digest {
        &self.review_policy
    }

    /// Exact proposal+review-policy context commitment.
    pub fn review_policy_context(&self) -> &Digest {
        &self.review_policy_context
    }

    /// Immutable governance rationale/evidence commitment for adoption.
    pub fn adoption_context(&self) -> &Digest {
        &self.adoption_context
    }

    /// Canonical bytes for the exact adoption statement.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewPolicyAdoptionError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(ADOPTION_STATEMENT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.review_policy)?;
        push_digest(&mut out, &self.review_policy_context)?;
        push_digest(&mut out, &self.adoption_context)?;
        Ok(out)
    }

    /// Stable identity for this exact adoption statement.
    pub fn statement_id(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<ProposalReviewPolicyAdoptionStatementId, ReviewPolicyAdoptionError> {
        Ok(ProposalReviewPolicyAdoptionStatementId::new(Digest::of_bytes(
            algorithm,
            &self.canonical_bytes()?,
        )))
    }
}

/// Positive result proving one exact project-policy-trusted authentication
/// endorsed one exact review-policy adoption statement and its principal was
/// structurally eligible for `ManagePolicy` at the supplied time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewPolicyAuthorityAttestationV1 {
    statement: ProposalReviewPolicyAdoptionStatementId,
    authority_principal: PrincipalId,
    observed_at_unix_ms: u64,
    trusted_authentication_evidence: Digest,
    verifier_identity: Digest,
    evidence_commitment: Digest,
}

impl ReviewPolicyAuthorityAttestationV1 {
    /// Exact adoption statement authenticated by this policy principal.
    pub fn statement(&self) -> &ProposalReviewPolicyAdoptionStatementId {
        &self.statement
    }

    /// Distinct `ManagePolicy` principal making this attestation.
    pub fn authority_principal(&self) -> &PrincipalId {
        &self.authority_principal
    }

    /// Caller-supplied time used for structural policy eligibility.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact project-policy-trusted Xenia authentication evidence.
    pub fn trusted_authentication_evidence(&self) -> &Digest {
        &self.trusted_authentication_evidence
    }

    /// Exact Xenia verifier identity trusted by the proposal's project policy.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Aggregate per-principal policy-attestation evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one project-policy-trusted Xenia authentication to one exact review
/// policy adoption statement and require `Capability::ManagePolicy`.
pub fn bind_review_policy_authority_attestation_v1(
    proposal: &ChangeProposal,
    policy: &ReviewAcceptancePolicyV1,
    policy_context: &ProposalReviewPolicyContextV1,
    statement: &ProposalReviewPolicyAdoptionStatementV1,
    trusted_authentication: &ProjectPolicyTrustedXeniaAuthenticationV1,
    authority_epoch: &AuthorityEpoch,
    observed_at_unix_ms: u64,
) -> Result<ReviewPolicyAuthorityAttestationV1, ReviewPolicyAdoptionError> {
    validate_statement(proposal, policy, policy_context, statement)?;
    if authority_epoch.project() != proposal.project() {
        return Err(ReviewPolicyAdoptionError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch {
        return Err(ReviewPolicyAdoptionError::AuthorityEpochMismatch);
    }

    let authentication = trusted_authentication.authentication();
    let request = authentication.request();
    if request.project() != proposal.project() {
        return Err(ReviewPolicyAdoptionError::ProjectMismatch);
    }
    if request.authority_epoch() != &expected_epoch {
        return Err(ReviewPolicyAdoptionError::AuthorityEpochMismatch);
    }
    if request.capability() != Capability::ManagePolicy {
        return Err(ReviewPolicyAdoptionError::WrongAuthenticationCapability);
    }
    if request.principal() != authentication.binding().forge_principal() {
        return Err(ReviewPolicyAdoptionError::AuthorityPrincipalMismatch);
    }
    if trusted_authentication.project_policy() != proposal.project_policy() {
        return Err(ReviewPolicyAdoptionError::ProjectPolicyMismatch);
    }

    let expected_statement = statement.statement_id(request.action_subject().algorithm())?;
    if request.action_subject() != expected_statement.commitment() {
        return Err(ReviewPolicyAdoptionError::AuthenticationSubjectMismatch);
    }
    if !authority_epoch.is_principal_eligible(
        request.principal(),
        Capability::ManagePolicy,
        observed_at_unix_ms,
    ) {
        return Err(ReviewPolicyAdoptionError::PolicyPrincipalNotEligible);
    }

    let algorithm = trusted_authentication.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(POLICY_ATTESTATION_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, expected_statement.commitment())?;
    push_digest(&mut out, request.principal().commitment())?;
    out.extend_from_slice(&observed_at_unix_ms.to_be_bytes());
    push_digest(&mut out, trusted_authentication.evidence_commitment())?;
    push_digest(&mut out, trusted_authentication.verifier_identity())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ReviewPolicyAuthorityAttestationV1 {
        statement: expected_statement,
        authority_principal: request.principal().clone(),
        observed_at_unix_ms,
        trusted_authentication_evidence: trusted_authentication.evidence_commitment().clone(),
        verifier_identity: trusted_authentication.verifier_identity().clone(),
        evidence_commitment,
    })
}

/// One normalized `ManagePolicy` principal counted by a successful policy quorum.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CountedReviewPolicyAuthorityV1 {
    principal: PrincipalId,
    observed_at_unix_ms: u64,
    attestation_evidence: Digest,
}

impl CountedReviewPolicyAuthorityV1 {
    /// Distinct policy-authority principal.
    pub fn principal(&self) -> &PrincipalId {
        &self.principal
    }

    /// Original structural observation time for the attestation.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact attestation evidence counted by the quorum.
    pub fn attestation_evidence(&self) -> &Digest {
        &self.attestation_evidence
    }
}

/// Positive result proving the exact `ManagePolicy` threshold accepted one exact
/// proposal review-policy adoption statement at one common supplied time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProposalReviewPolicyAuthorityQuorumV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    review_policy: Digest,
    review_policy_context: Digest,
    statement: ProposalReviewPolicyAdoptionStatementId,
    quorum_observed_at_unix_ms: u64,
    threshold: u16,
    authorizers: Vec<CountedReviewPolicyAuthorityV1>,
    evidence_commitment: Digest,
}

impl ProposalReviewPolicyAuthorityQuorumV1 {
    /// Project whose policy threshold accepted the review policy.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal governed by the adopted review policy.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch used to evaluate `ManagePolicy`.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project policy governing Xenia provider trust.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact adopted review-policy commitment.
    pub fn review_policy(&self) -> &Digest {
        &self.review_policy
    }

    /// Exact proposal+review-policy context commitment.
    pub fn review_policy_context(&self) -> &Digest {
        &self.review_policy_context
    }

    /// Exact adoption statement accepted by the quorum.
    pub fn statement(&self) -> &ProposalReviewPolicyAdoptionStatementId {
        &self.statement
    }

    /// Common caller-supplied observation time used for quorum evaluation.
    pub const fn quorum_observed_at_unix_ms(&self) -> u64 {
        self.quorum_observed_at_unix_ms
    }

    /// Exact distinct `ManagePolicy` threshold required by the authority epoch.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    /// Canonically sorted distinct policy principals counted by the quorum.
    pub fn authorizers(&self) -> &[CountedReviewPolicyAuthorityV1] {
        &self.authorizers
    }

    /// Aggregate review-policy-adoption quorum evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate the exact distinct `ManagePolicy` threshold over one exact review-
/// policy adoption statement.
pub fn evaluate_review_policy_authority_quorum_v1(
    proposal: &ChangeProposal,
    policy: &ReviewAcceptancePolicyV1,
    policy_context: &ProposalReviewPolicyContextV1,
    statement: &ProposalReviewPolicyAdoptionStatementV1,
    authority_epoch: &AuthorityEpoch,
    attestations: &[ReviewPolicyAuthorityAttestationV1],
    quorum_observed_at_unix_ms: u64,
) -> Result<ProposalReviewPolicyAuthorityQuorumV1, ReviewPolicyAdoptionError> {
    validate_statement(proposal, policy, policy_context, statement)?;
    if authority_epoch.project() != proposal.project() {
        return Err(ReviewPolicyAdoptionError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch {
        return Err(ReviewPolicyAdoptionError::AuthorityEpochMismatch);
    }
    if !authority_epoch.is_valid_at(quorum_observed_at_unix_ms) {
        return Err(ReviewPolicyAdoptionError::AuthorityEpochNotValidAtQuorumTime);
    }
    let threshold = authority_epoch
        .threshold_for(Capability::ManagePolicy)
        .ok_or(ReviewPolicyAdoptionError::MissingManagePolicyThreshold)?;

    let expected_statement =
        statement.statement_id(policy_context.review_acceptance_policy().algorithm())?;
    let mut seen = BTreeSet::new();
    let mut authorizers = Vec::with_capacity(attestations.len());
    for attestation in attestations {
        if attestation.statement() != &expected_statement {
            return Err(ReviewPolicyAdoptionError::AttestationStatementMismatch);
        }
        if attestation.observed_at_unix_ms() > quorum_observed_at_unix_ms {
            return Err(ReviewPolicyAdoptionError::PolicyObservedAfterQuorumTime(
                attestation.authority_principal().clone(),
            ));
        }
        if !authority_epoch.is_principal_eligible(
            attestation.authority_principal(),
            Capability::ManagePolicy,
            quorum_observed_at_unix_ms,
        ) {
            return Err(ReviewPolicyAdoptionError::PolicyPrincipalNotEligibleAtQuorumTime(
                attestation.authority_principal().clone(),
            ));
        }
        if !seen.insert(attestation.authority_principal().clone()) {
            return Err(ReviewPolicyAdoptionError::DuplicatePolicyPrincipal(
                attestation.authority_principal().clone(),
            ));
        }
        authorizers.push(CountedReviewPolicyAuthorityV1 {
            principal: attestation.authority_principal().clone(),
            observed_at_unix_ms: attestation.observed_at_unix_ms(),
            attestation_evidence: attestation.evidence_commitment().clone(),
        });
    }

    authorizers.sort_by(|a, b| a.principal.cmp(&b.principal));
    if authorizers.len() < usize::from(threshold) {
        return Err(ReviewPolicyAdoptionError::UnderManagePolicyThreshold {
            required: threshold,
            actual: authorizers.len(),
        });
    }

    let proposal_id = proposal.proposal_id(statement.proposal().commitment().algorithm())?;
    let policy_digest = policy.digest(statement.review_policy().algorithm())?;
    let context_digest = policy_context.digest(statement.review_policy_context().algorithm())?;
    let algorithm = expected_statement.commitment().algorithm();
    let count = u32::try_from(authorizers.len()).map_err(|_| {
        ReviewPolicyAdoptionError::CanonicalFieldTooLarge {
            field: "authorizers",
            len: authorizers.len(),
            max: u32::MAX as usize,
        }
    })?;

    let mut out = Vec::new();
    out.extend_from_slice(POLICY_QUORUM_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, proposal_id.commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    push_digest(&mut out, proposal.project_policy())?;
    push_digest(&mut out, &policy_digest)?;
    push_digest(&mut out, &context_digest)?;
    push_digest(&mut out, expected_statement.commitment())?;
    out.extend_from_slice(&quorum_observed_at_unix_ms.to_be_bytes());
    out.extend_from_slice(&threshold.to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for authorizer in &authorizers {
        push_digest(&mut out, authorizer.principal.commitment())?;
        out.extend_from_slice(&authorizer.observed_at_unix_ms.to_be_bytes());
        push_digest(&mut out, &authorizer.attestation_evidence)?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProposalReviewPolicyAuthorityQuorumV1 {
        project: proposal.project().clone(),
        proposal: proposal_id,
        authority_epoch: expected_epoch,
        project_policy: proposal.project_policy().clone(),
        review_policy: policy_digest,
        review_policy_context: context_digest,
        statement: expected_statement,
        quorum_observed_at_unix_ms,
        threshold,
        authorizers,
        evidence_commitment,
    })
}

fn validate_policy_context(
    proposal: &ChangeProposal,
    policy: &ReviewAcceptancePolicyV1,
    policy_context: &ProposalReviewPolicyContextV1,
    algorithm: DigestAlgorithm,
) -> Result<(), ReviewPolicyAdoptionError> {
    if policy.project() != proposal.project() || policy_context.project() != proposal.project() {
        return Err(ReviewPolicyAdoptionError::ProjectMismatch);
    }
    let expected_proposal = proposal.proposal_id(policy_context.proposal().commitment().algorithm())?;
    if policy_context.proposal() != &expected_proposal {
        return Err(ReviewPolicyAdoptionError::ProposalMismatch);
    }
    let expected_policy = policy.digest(policy_context.review_acceptance_policy().algorithm())?;
    if policy_context.review_acceptance_policy() != &expected_policy {
        return Err(ReviewPolicyAdoptionError::ReviewPolicyMismatch);
    }
    let rebuilt = ProposalReviewPolicyContextV1::new(proposal, policy, algorithm)?;
    if rebuilt != *policy_context {
        return Err(ReviewPolicyAdoptionError::ReviewPolicyContextMismatch);
    }
    Ok(())
}

fn validate_statement(
    proposal: &ChangeProposal,
    policy: &ReviewAcceptancePolicyV1,
    policy_context: &ProposalReviewPolicyContextV1,
    statement: &ProposalReviewPolicyAdoptionStatementV1,
) -> Result<(), ReviewPolicyAdoptionError> {
    validate_policy_context(
        proposal,
        policy,
        policy_context,
        statement.review_policy().algorithm(),
    )?;
    if statement.project() != proposal.project() {
        return Err(ReviewPolicyAdoptionError::ProjectMismatch);
    }
    let expected_proposal = proposal.proposal_id(statement.proposal().commitment().algorithm())?;
    if statement.proposal() != &expected_proposal {
        return Err(ReviewPolicyAdoptionError::ProposalMismatch);
    }
    let expected_policy = policy.digest(statement.review_policy().algorithm())?;
    if statement.review_policy() != &expected_policy {
        return Err(ReviewPolicyAdoptionError::ReviewPolicyMismatch);
    }
    let expected_context = policy_context.digest(statement.review_policy_context().algorithm())?;
    if statement.review_policy_context() != &expected_context {
        return Err(ReviewPolicyAdoptionError::ReviewPolicyContextMismatch);
    }
    Ok(())
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ReviewPolicyAdoptionError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewPolicyAdoptionError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewPolicyAdoptionError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewPolicyAdoptionError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewPolicyAdoptionError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ReviewPolicyAdoptionError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Proposal review-policy adoption failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReviewPolicyAdoptionError {
    /// Proposal, policy, context, authority, or authentication projects differ.
    #[error("proposal review-policy adoption project mismatch")]
    ProjectMismatch,
    /// Policy context or statement names another proposal.
    #[error("proposal review-policy adoption proposal mismatch")]
    ProposalMismatch,
    /// Policy context or statement names another review policy.
    #[error("proposal review-policy adoption review-policy mismatch")]
    ReviewPolicyMismatch,
    /// Policy context commitment differs from the exact rebuilt context.
    #[error("proposal review-policy context mismatch")]
    ReviewPolicyContextMismatch,
    /// Authority epoch differs from the proposal's exact authority epoch.
    #[error("proposal review-policy adoption authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Authentication is not scoped to `ManagePolicy`.
    #[error("review-policy adoption authentication is not scoped to ManagePolicy")]
    WrongAuthenticationCapability,
    /// Authentication request principal and principal binding disagree.
    #[error("review-policy adoption policy principal mismatch")]
    AuthorityPrincipalMismatch,
    /// Trusted authentication uses another project-policy state.
    #[error("review-policy adoption project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Authentication action subject differs from the exact adoption statement.
    #[error("authentication action subject does not match review-policy adoption statement")]
    AuthenticationSubjectMismatch,
    /// Principal was not structurally eligible for `ManagePolicy` at attestation time.
    #[error("principal is not eligible for ManagePolicy at attestation time")]
    PolicyPrincipalNotEligible,
    /// Authority epoch is not valid at common policy-quorum time.
    #[error("authority epoch is not valid at review-policy quorum time")]
    AuthorityEpochNotValidAtQuorumTime,
    /// Exact authority epoch has no `ManagePolicy` threshold.
    #[error("authority epoch has no ManagePolicy threshold")]
    MissingManagePolicyThreshold,
    /// Attestation names another policy-adoption statement.
    #[error("policy attestation names another review-policy adoption statement")]
    AttestationStatementMismatch,
    /// Same policy principal appeared more than once.
    #[error("duplicate review-policy principal: {0}")]
    DuplicatePolicyPrincipal(PrincipalId),
    /// Attestation was observed after the claimed common policy-quorum time.
    #[error("policy principal {0} was observed after quorum time")]
    PolicyObservedAfterQuorumTime(PrincipalId),
    /// Policy principal was not eligible at the common quorum time.
    #[error("policy principal {0} is not eligible at quorum time")]
    PolicyPrincipalNotEligibleAtQuorumTime(PrincipalId),
    /// Distinct policy-principal count is below threshold.
    #[error("ManagePolicy quorum below threshold: {actual} < {required}")]
    UnderManagePolicyThreshold {
        /// Required distinct policy principals.
        required: u16,
        /// Supplied distinct positive policy principals.
        actual: usize,
    },
    /// Unsupported protocol version.
    #[error("unsupported Forge review-policy adoption protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Canonical field exceeded the v1 encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum length.
        max: usize,
    },
    /// Authority validation/canonicalization failure.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
    /// Proposal validation/canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Review-acceptance policy validation/canonicalization failure.
    #[error(transparent)]
    ReviewPolicy(#[from] ReviewAcceptancePolicyError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authority::{AuthorityEpochParts, CapabilityRule, PrincipalGrant};
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review_acceptance_policy::{
        ProposerReviewRuleV1, RequestChangesRuleV1,
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

    struct Context {
        authority: AuthorityEpoch,
        proposal: ChangeProposal,
        policy: ReviewAcceptancePolicyV1,
        policy_context: ProposalReviewPolicyContextV1,
        statement: ProposalReviewPolicyAdoptionStatementV1,
    }

    fn context() -> Context {
        let project = project();
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![TrustedProviderVerifierV1::new(digest(0x31), digest(0x32))],
        )
        .unwrap();
        let project_policy = ProjectPolicyStateV1::new(
            project.clone(),
            0,
            None,
            &provider_trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let authority = AuthorityEpoch::new(AuthorityEpochParts {
            project: project.clone(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(5_000),
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(principal(0x20), [Capability::ManagePolicy]).unwrap(),
                PrincipalGrant::new(principal(0x21), [Capability::ManagePolicy]).unwrap(),
                PrincipalGrant::new(principal(0x22), [Capability::ManagePolicy]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ManagePolicy, 2).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap();
        let repository_policy =
            RepositoryPolicyState::new(project.clone(), 0, None, digest(0x40)).unwrap();
        let proposal = ChangeProposal::new(
            principal(0x30),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x41; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x42; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x43; 20]).unwrap(),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let policy = ReviewAcceptancePolicyV1::new(
            project,
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![principal(0x60)],
        )
        .unwrap();
        let policy_context = ProposalReviewPolicyContextV1::new(
            &proposal,
            &policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let statement = ProposalReviewPolicyAdoptionStatementV1::new(
            &proposal,
            &policy,
            &policy_context,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        Context {
            authority,
            proposal,
            policy,
            policy_context,
            statement,
        }
    }

    fn fake_attestation(
        statement: &ProposalReviewPolicyAdoptionStatementId,
        principal: PrincipalId,
        observed_at_unix_ms: u64,
        marker: u8,
    ) -> ReviewPolicyAuthorityAttestationV1 {
        ReviewPolicyAuthorityAttestationV1 {
            statement: statement.clone(),
            authority_principal: principal,
            observed_at_unix_ms,
            trusted_authentication_evidence: digest(marker),
            verifier_identity: digest(marker.wrapping_add(1)),
            evidence_commitment: digest(marker.wrapping_add(2)),
        }
    }

    #[test]
    fn quorum_uses_manage_policy_not_manage_authority_threshold() {
        let context = context();
        let statement = context.statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let a = fake_attestation(&statement, principal(0x20), 1_400, 0x80);
        let b = fake_attestation(&statement, principal(0x21), 1_450, 0x90);
        let result = evaluate_review_policy_authority_quorum_v1(
            &context.proposal,
            &context.policy,
            &context.policy_context,
            &context.statement,
            &context.authority,
            &[b, a],
            1_500,
        )
        .unwrap();
        assert_eq!(result.threshold(), 2);
        assert_eq!(result.authorizers().len(), 2);
    }

    #[test]
    fn under_manage_policy_threshold_fails_closed() {
        let context = context();
        let statement = context.statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let a = fake_attestation(&statement, principal(0x20), 1_400, 0x80);
        assert_eq!(
            evaluate_review_policy_authority_quorum_v1(
                &context.proposal,
                &context.policy,
                &context.policy_context,
                &context.statement,
                &context.authority,
                &[a],
                1_500,
            )
            .unwrap_err(),
            ReviewPolicyAdoptionError::UnderManagePolicyThreshold {
                required: 2,
                actual: 1,
            }
        );
    }

    #[test]
    fn duplicate_policy_principal_is_rejected() {
        let context = context();
        let statement = context.statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let a = fake_attestation(&statement, principal(0x20), 1_400, 0x80);
        assert_eq!(
            evaluate_review_policy_authority_quorum_v1(
                &context.proposal,
                &context.policy,
                &context.policy_context,
                &context.statement,
                &context.authority,
                &[a.clone(), a],
                1_500,
            )
            .unwrap_err(),
            ReviewPolicyAdoptionError::DuplicatePolicyPrincipal(principal(0x20))
        );
    }

    #[test]
    fn future_policy_attestation_cannot_count_retroactively() {
        let context = context();
        let statement = context.statement.statement_id(DigestAlgorithm::Sha256).unwrap();
        let a = fake_attestation(&statement, principal(0x20), 1_400, 0x80);
        let b = fake_attestation(&statement, principal(0x21), 1_600, 0x90);
        assert_eq!(
            evaluate_review_policy_authority_quorum_v1(
                &context.proposal,
                &context.policy,
                &context.policy_context,
                &context.statement,
                &context.authority,
                &[a, b],
                1_500,
            )
            .unwrap_err(),
            ReviewPolicyAdoptionError::PolicyObservedAfterQuorumTime(principal(0x21))
        );
    }

    #[test]
    fn changing_review_policy_changes_adoption_statement_identity() {
        let context = context();
        let changed_policy = ReviewAcceptancePolicyV1::new(
            project(),
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![principal(0x60)],
        )
        .unwrap();
        let changed_context = ProposalReviewPolicyContextV1::new(
            &context.proposal,
            &changed_policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let changed_statement = ProposalReviewPolicyAdoptionStatementV1::new(
            &context.proposal,
            &changed_policy,
            &changed_context,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(
            context.statement.statement_id(DigestAlgorithm::Sha256).unwrap(),
            changed_statement.statement_id(DigestAlgorithm::Sha256).unwrap()
        );
    }
}

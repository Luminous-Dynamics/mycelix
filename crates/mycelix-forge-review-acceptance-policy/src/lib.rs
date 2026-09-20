// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed review-acceptance policy and proposal-review policy context for Mycelix Forge.
//!
//! This crate deliberately does **not** modify `ProjectPolicyStateV1` or
//! `ChangeProposal` v1 canonical bytes. Forge core currently supports protocol
//! version 1 only, so reinterpreting an existing v1 commitment would make old
//! proposal identities ambiguous.
//!
//! Instead this tranche defines an explicit review policy and a canonical
//! proposal+policy context commitment suitable for use as
//! `ReviewRevisionV1.review_context`.
//!
//! ```text
//! ChangeProposalId
//! + exact ReviewAcceptancePolicyV1
//!     -> ProposalReviewPolicyContextV1
//!     -> review_context commitment
//! ```
//!
//! Changing the review policy therefore changes the context commitment and, once
//! used by `ReviewRevisionV1`, changes the authenticated review revision id.
//! This crate does not itself prove that the policy was project-authorized or
//! that a review actually used the context.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::PrincipalId;
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const REVIEW_ACCEPTANCE_POLICY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-acceptance-policy/v1\0";
const PROPOSAL_REVIEW_POLICY_CONTEXT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/proposal-review-policy-context/v1\0";
const MAX_REQUIRED_APPROVERS_V1: usize = 1024;

/// How a checkpoint-head `RequestChanges` state participates in later review
/// acceptance evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RequestChangesRuleV1 {
    /// Any admitted current `RequestChanges` head blocks review acceptance.
    BlocksAcceptance,
    /// `RequestChanges` never counts as approval but is not by itself a blocker.
    NonCountingOnly,
}

impl RequestChangesRuleV1 {
    const fn code(self) -> u8 {
        match self {
            Self::BlocksAcceptance => 1,
            Self::NonCountingOnly => 2,
        }
    }
}

/// How the proposal author may participate in review.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProposerReviewRuleV1 {
    /// The proposer may review and an `Approve` may count toward threshold.
    MayCountTowardThreshold,
    /// The proposer may leave review state, but their approval must not count
    /// toward the approval threshold.
    MayReviewButDoesNotCount,
    /// The proposer must not supply a review accepted by review policy.
    Prohibited,
}

impl ProposerReviewRuleV1 {
    const fn code(self) -> u8 {
        match self {
            Self::MayCountTowardThreshold => 1,
            Self::MayReviewButDoesNotCount => 2,
            Self::Prohibited => 3,
        }
    }
}

/// Immutable project-scoped rules for interpreting a witnessed review checkpoint.
///
/// The authority epoch remains the source of the numeric `ReviewSource`
/// threshold. This policy does not duplicate that threshold; it defines how
/// exact review states are interpreted around it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReviewAcceptancePolicyV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    request_changes_rule: RequestChangesRuleV1,
    proposer_review_rule: ProposerReviewRuleV1,
    required_approvers: Vec<PrincipalId>,
}

impl ReviewAcceptancePolicyV1 {
    /// Construct a canonical review-acceptance policy.
    ///
    /// Required approvers are sorted canonically. Duplicate principals are
    /// rejected rather than silently deduplicated.
    pub fn new(
        project: ProjectIdentity,
        request_changes_rule: RequestChangesRuleV1,
        proposer_review_rule: ProposerReviewRuleV1,
        mut required_approvers: Vec<PrincipalId>,
    ) -> Result<Self, ReviewAcceptancePolicyError> {
        if required_approvers.len() > MAX_REQUIRED_APPROVERS_V1 {
            return Err(ReviewAcceptancePolicyError::TooManyRequiredApprovers {
                actual: required_approvers.len(),
                max: MAX_REQUIRED_APPROVERS_V1,
            });
        }
        required_approvers.sort();
        if let Some(pair) = required_approvers
            .windows(2)
            .find(|pair| pair[0] == pair[1])
        {
            return Err(ReviewAcceptancePolicyError::DuplicateRequiredApprover(
                pair[0].clone(),
            ));
        }
        Self::from_canonical_parts(
            project,
            request_changes_rule,
            proposer_review_rule,
            required_approvers,
        )
    }

    fn from_canonical_parts(
        project: ProjectIdentity,
        request_changes_rule: RequestChangesRuleV1,
        proposer_review_rule: ProposerReviewRuleV1,
        required_approvers: Vec<PrincipalId>,
    ) -> Result<Self, ReviewAcceptancePolicyError> {
        if required_approvers.len() > MAX_REQUIRED_APPROVERS_V1 {
            return Err(ReviewAcceptancePolicyError::TooManyRequiredApprovers {
                actual: required_approvers.len(),
                max: MAX_REQUIRED_APPROVERS_V1,
            });
        }
        if required_approvers
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err(ReviewAcceptancePolicyError::NonCanonicalRequiredApprovers);
        }
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            request_changes_rule,
            proposer_review_rule,
            required_approvers,
        })
    }

    /// Project whose review semantics are described.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact `RequestChanges` interpretation.
    pub const fn request_changes_rule(&self) -> RequestChangesRuleV1 {
        self.request_changes_rule
    }

    /// Exact proposer/self-review rule.
    pub const fn proposer_review_rule(&self) -> ProposerReviewRuleV1 {
        self.proposer_review_rule
    }

    /// Canonically ordered principals whose checkpoint-head state must later be
    /// evaluated as required approval.
    pub fn required_approvers(&self) -> &[PrincipalId] {
        &self.required_approvers
    }

    /// Whether a principal is explicitly required by this policy.
    pub fn requires_approver(&self, principal: &PrincipalId) -> bool {
        self.required_approvers.binary_search(principal).is_ok()
    }

    /// Canonical v1 policy bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewAcceptancePolicyError> {
        ensure_v1(self.version)?;
        if self
            .required_approvers
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err(ReviewAcceptancePolicyError::NonCanonicalRequiredApprovers);
        }
        let count = u16::try_from(self.required_approvers.len()).map_err(|_| {
            ReviewAcceptancePolicyError::CanonicalFieldTooLarge {
                field: "required_approvers",
                len: self.required_approvers.len(),
                max: u16::MAX as usize,
            }
        })?;
        let mut out = Vec::new();
        out.extend_from_slice(REVIEW_ACCEPTANCE_POLICY_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        out.push(self.request_changes_rule.code());
        out.push(self.proposer_review_rule.code());
        out.extend_from_slice(&count.to_be_bytes());
        for approver in &self.required_approvers {
            push_digest(&mut out, approver.commitment())?;
        }
        Ok(out)
    }

    /// Stable identity of the exact policy.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ReviewAcceptancePolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ReviewAcceptancePolicyV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WirePolicy {
            version: ProtocolVersion,
            project: ProjectIdentity,
            request_changes_rule: RequestChangesRuleV1,
            proposer_review_rule: ProposerReviewRuleV1,
            required_approvers: Vec<PrincipalId>,
        }

        let wire = WirePolicy::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_canonical_parts(
            wire.project,
            wire.request_changes_rule,
            wire.proposer_review_rule,
            wire.required_approvers,
        )
        .map_err(D::Error::custom)
    }
}

/// Exact proposal+review-policy context intended to become an authenticated
/// review revision's immutable `review_context` commitment.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProposalReviewPolicyContextV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    review_acceptance_policy: Digest,
}

impl ProposalReviewPolicyContextV1 {
    /// Bind one exact proposal to one exact review-acceptance policy.
    pub fn new(
        proposal: &ChangeProposal,
        policy: &ReviewAcceptancePolicyV1,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ReviewAcceptancePolicyError> {
        if policy.project() != proposal.project() {
            return Err(ReviewAcceptancePolicyError::PolicyProjectMismatch);
        }
        if policy.proposer_review_rule() == ProposerReviewRuleV1::Prohibited
            && policy.requires_approver(proposal.proposer())
        {
            return Err(ReviewAcceptancePolicyError::ProhibitedProposerIsRequiredApprover);
        }
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project: proposal.project().clone(),
            proposal: proposal.proposal_id(commitment_algorithm)?,
            review_acceptance_policy: policy.digest(commitment_algorithm)?,
        })
    }

    /// Project shared by the proposal and policy.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal being reviewed.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact review-acceptance policy commitment.
    pub fn review_acceptance_policy(&self) -> &Digest {
        &self.review_acceptance_policy
    }

    /// Canonical bytes for the proposal-review policy context.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewAcceptancePolicyError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(PROPOSAL_REVIEW_POLICY_CONTEXT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.review_acceptance_policy)?;
        Ok(out)
    }

    /// Stable context commitment suitable for `ReviewRevisionV1.review_context`.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ReviewAcceptancePolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ReviewAcceptancePolicyError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewAcceptancePolicyError::UnsupportedProtocolVersion(
            version.get(),
        ))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewAcceptancePolicyError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ReviewAcceptancePolicyError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewAcceptancePolicyError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ReviewAcceptancePolicyError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Review-acceptance policy validation/canonicalization failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewAcceptancePolicyError {
    /// Policy exceeded the v1 required-approver bound.
    #[error("too many required approvers: {actual} > {max}")]
    TooManyRequiredApprovers {
        /// Supplied count.
        actual: usize,
        /// Maximum v1 count.
        max: usize,
    },
    /// The same required approver was supplied more than once.
    #[error("duplicate required approver: {0}")]
    DuplicateRequiredApprover(PrincipalId),
    /// Serialized required approvers were not in strict canonical order.
    #[error("required approvers are not in canonical strict order")]
    NonCanonicalRequiredApprovers,
    /// Policy and proposal belong to different projects.
    #[error("review-acceptance policy belongs to another project")]
    PolicyProjectMismatch,
    /// A policy cannot both prohibit proposer review and require the proposer to approve.
    #[error("proposer review is prohibited but proposer is a required approver")]
    ProhibitedProposerIsRequiredApprover,
    /// Unsupported protocol version.
    #[error("unsupported Forge review-acceptance policy protocol version: {0}")]
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
    /// Proposal identity/canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
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

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn project(byte: u8) -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([byte; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn proposal(project: ProjectIdentity, proposer: PrincipalId, marker: u8) -> ChangeProposal {
        let authority = AuthorityEpoch::new(AuthorityEpochParts {
            project: project.clone(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(5_000),
            grants: vec![PrincipalGrant::new(
                principal(0x20),
                [Capability::ManageAuthority, Capability::ReviewSource],
            )
            .unwrap()],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap();
        let trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![TrustedProviderVerifierV1::new(digest(0x30), digest(0x31))],
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
            RepositoryPolicyState::new(project, 0, None, digest(0x40)).unwrap();
        ChangeProposal::new(
            proposer,
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![marker; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![marker.wrapping_add(1); 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![marker.wrapping_add(2); 20]).unwrap(),
            digest(marker.wrapping_add(3)),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn policy(project: ProjectIdentity) -> ReviewAcceptancePolicyV1 {
        ReviewAcceptancePolicyV1::new(
            project,
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![principal(0x22), principal(0x21)],
        )
        .unwrap()
    }

    #[test]
    fn required_approvers_are_canonical_and_input_order_invariant() {
        let p = project(0x11);
        let a = ReviewAcceptancePolicyV1::new(
            p.clone(),
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![principal(0x22), principal(0x21)],
        )
        .unwrap();
        let b = ReviewAcceptancePolicyV1::new(
            p,
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![principal(0x21), principal(0x22)],
        )
        .unwrap();
        assert_eq!(a, b);
        assert_eq!(
            a.digest(DigestAlgorithm::Sha256).unwrap(),
            b.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn duplicate_required_approver_fails_closed() {
        let error = ReviewAcceptancePolicyV1::new(
            project(0x11),
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::MayCountTowardThreshold,
            vec![principal(0x21), principal(0x21)],
        )
        .unwrap_err();
        assert_eq!(
            error,
            ReviewAcceptancePolicyError::DuplicateRequiredApprover(principal(0x21))
        );
    }

    #[test]
    fn changing_request_changes_semantics_changes_policy_identity() {
        let p = project(0x11);
        let blocking = policy(p.clone());
        let non_blocking = ReviewAcceptancePolicyV1::new(
            p,
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![principal(0x21), principal(0x22)],
        )
        .unwrap();
        assert_ne!(
            blocking.digest(DigestAlgorithm::Sha256).unwrap(),
            non_blocking.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn changing_proposer_semantics_changes_policy_identity() {
        let p = project(0x11);
        let first = policy(p.clone());
        let second = ReviewAcceptancePolicyV1::new(
            p,
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::Prohibited,
            vec![principal(0x21), principal(0x22)],
        )
        .unwrap();
        assert_ne!(
            first.digest(DigestAlgorithm::Sha256).unwrap(),
            second.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn policy_change_changes_authenticated_review_context() {
        let p = project(0x11);
        let proposal = proposal(p.clone(), principal(0x50), 0x60);
        let first_policy = policy(p.clone());
        let second_policy = ReviewAcceptancePolicyV1::new(
            p,
            RequestChangesRuleV1::NonCountingOnly,
            ProposerReviewRuleV1::MayReviewButDoesNotCount,
            vec![principal(0x21), principal(0x22)],
        )
        .unwrap();
        let first = ProposalReviewPolicyContextV1::new(
            &proposal,
            &first_policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let second = ProposalReviewPolicyContextV1::new(
            &proposal,
            &second_policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(
            first.digest(DigestAlgorithm::Sha256).unwrap(),
            second.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn proposal_change_changes_review_policy_context() {
        let p = project(0x11);
        let policy = policy(p.clone());
        let first = proposal(p.clone(), principal(0x50), 0x60);
        let second = proposal(p, principal(0x50), 0x70);
        let first_context = ProposalReviewPolicyContextV1::new(
            &first,
            &policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let second_context = ProposalReviewPolicyContextV1::new(
            &second,
            &policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(
            first_context.digest(DigestAlgorithm::Sha256).unwrap(),
            second_context.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn cross_project_policy_is_rejected() {
        let proposal = proposal(project(0x11), principal(0x50), 0x60);
        let foreign = policy(project(0x12));
        assert_eq!(
            ProposalReviewPolicyContextV1::new(
                &proposal,
                &foreign,
                DigestAlgorithm::Sha256,
            )
            .unwrap_err(),
            ReviewAcceptancePolicyError::PolicyProjectMismatch
        );
    }

    #[test]
    fn prohibited_proposer_cannot_also_be_required_approver() {
        let p = project(0x11);
        let proposer = principal(0x50);
        let proposal = proposal(p.clone(), proposer.clone(), 0x60);
        let policy = ReviewAcceptancePolicyV1::new(
            p,
            RequestChangesRuleV1::BlocksAcceptance,
            ProposerReviewRuleV1::Prohibited,
            vec![proposer],
        )
        .unwrap();
        assert_eq!(
            ProposalReviewPolicyContextV1::new(
                &proposal,
                &policy,
                DigestAlgorithm::Sha256,
            )
            .unwrap_err(),
            ReviewAcceptancePolicyError::ProhibitedProposerIsRequiredApprover
        );
    }

    #[test]
    fn serde_rejects_noncanonical_required_approvers() {
        let original = policy(project(0x11));
        let mut value = serde_json::to_value(&original).unwrap();
        value["required_approvers"].as_array_mut().unwrap().reverse();
        assert!(serde_json::from_value::<ReviewAcceptancePolicyV1>(value).is_err());
    }
}

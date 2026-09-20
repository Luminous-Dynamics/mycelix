// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable, non-authoritative protected-merge request subjects.
//!
//! This crate is the deliberate join *subject* between collaboration authority
//! and source/execution evidence. It does not turn source-evidence digests into
//! positive authority.
//!
//! ```text
//! exact ChangeProposal
//! + MergeProtectedReviewBasisQuorumV1
//! + explicit source/execution evidence references
//! + exact target ref/base/proposed/tree
//! + merge request nonce
//!     -> ProtectedMergeRequestV1
//! ```
//!
//! A later qualifier must rejoin every evidence reference to the corresponding
//! positive source/execution theorem and establish repository-tip currentness /
//! atomic transition semantics before any merge authorization exists.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_merge_protected_review_basis::MergeProtectedReviewBasisQuorumV1;
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const SOURCE_EVIDENCE_REFS_DOMAIN_V1: &[u8] =
    b"mycelix-forge/protected-merge-source-evidence-refs/v1\0";
const PROTECTED_MERGE_REQUEST_DOMAIN_V1: &[u8] = b"mycelix-forge/protected-merge-request/v1\0";

/// Fixed-width nonce size for one protected merge request.
pub const PROTECTED_MERGE_NONCE_LEN: usize = 32;

/// Git-source semantic verifier profile expected by a protected merge request.
///
/// This identifies the intended evidence contract; it does not prove that such
/// evidence exists or passed verification.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProposalSourceProfileV1 {
    /// FORGE-008B-style pinned Git object/type/tree/ancestry verification.
    #[serde(rename = "pinned-git-v1")]
    PinnedGitV1,
}

impl ProposalSourceProfileV1 {
    const fn code(self) -> u8 {
        match self {
            Self::PinnedGitV1 => 1,
        }
    }
}

/// Offline source/execution evidence profile expected by a protected merge
/// request.
///
/// This identifies the evidence contract only; it is not a qualification claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum OfflineExecutionProfileV1 {
    /// Frozen M0 OfflineEvidence v6 composition contract.
    #[serde(rename = "m0-offline-evidence-v6")]
    M0OfflineEvidenceV6,
}

impl OfflineExecutionProfileV1 {
    const fn code(self) -> u8 {
        match self {
            Self::M0OfflineEvidenceV6 => 1,
        }
    }
}

/// Explicit references to the source/execution evidence a later merge
/// authorization qualifier must rejoin to positive evidence objects.
///
/// This structure is intentionally deserializable and non-authoritative. A
/// caller can serialize arbitrary digests here; later qualification must
/// recompute and verify every one of them.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceExecutionEvidenceReferencesV1 {
    proposal_source_profile: ProposalSourceProfileV1,
    offline_execution_profile: OfflineExecutionProfileV1,
    repository_request: Digest,
    repository_verification: Digest,
    proposal_source: Digest,
    source_state: Digest,
    offline_evidence: Digest,
    execution_subject: Digest,
}

impl SourceExecutionEvidenceReferencesV1 {
    /// Construct exact source/execution evidence references.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        proposal_source_profile: ProposalSourceProfileV1,
        offline_execution_profile: OfflineExecutionProfileV1,
        repository_request: Digest,
        repository_verification: Digest,
        proposal_source: Digest,
        source_state: Digest,
        offline_evidence: Digest,
        execution_subject: Digest,
    ) -> Self {
        Self {
            proposal_source_profile,
            offline_execution_profile,
            repository_request,
            repository_verification,
            proposal_source,
            source_state,
            offline_evidence,
            execution_subject,
        }
    }

    /// Required proposal-source verifier profile.
    pub const fn proposal_source_profile(&self) -> ProposalSourceProfileV1 {
        self.proposal_source_profile
    }

    /// Required offline source/execution evidence profile.
    pub const fn offline_execution_profile(&self) -> OfflineExecutionProfileV1 {
        self.offline_execution_profile
    }

    /// Exact repository-verification request digest to rejoin.
    pub fn repository_request(&self) -> &Digest {
        &self.repository_request
    }

    /// Exact positive repository-verification evidence digest to rejoin.
    pub fn repository_verification(&self) -> &Digest {
        &self.repository_verification
    }

    /// Exact proposal-source qualification evidence digest to rejoin.
    pub fn proposal_source(&self) -> &Digest {
        &self.proposal_source
    }

    /// Exact Git/object-store/source-state commitment inspected by source
    /// verification.
    pub fn source_state(&self) -> &Digest {
        &self.source_state
    }

    /// Exact OfflineEvidence aggregate commitment to rejoin.
    pub fn offline_evidence(&self) -> &Digest {
        &self.offline_evidence
    }

    /// Exact hermetic execution subject carried by the OfflineEvidence result.
    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    /// Canonical v1 bytes for these non-authoritative evidence references.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedMergeRequestError> {
        let mut out = Vec::new();
        out.extend_from_slice(SOURCE_EVIDENCE_REFS_DOMAIN_V1);
        out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
        out.push(self.proposal_source_profile.code());
        out.push(self.offline_execution_profile.code());
        push_digest(&mut out, &self.repository_request)?;
        push_digest(&mut out, &self.repository_verification)?;
        push_digest(&mut out, &self.proposal_source)?;
        push_digest(&mut out, &self.source_state)?;
        push_digest(&mut out, &self.offline_evidence)?;
        push_digest(&mut out, &self.execution_subject)?;
        Ok(out)
    }

    /// Stable commitment to the exact evidence-reference set.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedMergeRequestError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Stable identifier for one exact protected merge request.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ProtectedMergeRequestId(Digest);

impl ProtectedMergeRequestId {
    /// Algorithm-qualified request commitment.
    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

/// Exact, portable merge-attempt subject.
///
/// This type is deliberately **not** a merge authorization. Construction from
/// live inputs proves that the protected review basis and proposal agree on
/// project/policy identity, while source evidence remains a set of explicit
/// references awaiting later positive qualification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProtectedMergeRequestV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    target_ref: RepositoryRef,
    expected_base: GitObjectId,
    proposed_revision: GitObjectId,
    resulting_tree: GitObjectId,
    protected_review_basis: Digest,
    source_execution_evidence: SourceExecutionEvidenceReferencesV1,
    merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
}

impl ProtectedMergeRequestV1 {
    /// Construct one exact request from a live proposal and exact
    /// `MergeProtected` review-basis quorum.
    pub fn new(
        proposal: &ChangeProposal,
        protected_review_basis: &MergeProtectedReviewBasisQuorumV1,
        source_execution_evidence: SourceExecutionEvidenceReferencesV1,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
    ) -> Result<Self, ProtectedMergeRequestError> {
        if protected_review_basis.project() != proposal.project() {
            return Err(ProtectedMergeRequestError::ProjectMismatch);
        }
        let expected_proposal =
            proposal.proposal_id(protected_review_basis.proposal().commitment().algorithm())?;
        if protected_review_basis.proposal() != &expected_proposal {
            return Err(ProtectedMergeRequestError::ProposalMismatch);
        }
        if protected_review_basis.authority_epoch() != proposal.authority_epoch() {
            return Err(ProtectedMergeRequestError::AuthorityEpochMismatch);
        }
        if protected_review_basis.project_policy() != proposal.project_policy() {
            return Err(ProtectedMergeRequestError::ProjectPolicyMismatch);
        }
        if protected_review_basis.repository_policy_state() != proposal.repository_policy_state() {
            return Err(ProtectedMergeRequestError::RepositoryPolicyMismatch);
        }

        Self::from_parts(
            proposal.project().clone(),
            expected_proposal,
            proposal.authority_epoch().clone(),
            proposal.project_policy().clone(),
            proposal.repository_policy_state().clone(),
            proposal.target_ref().clone(),
            proposal.base_revision().clone(),
            proposal.proposed_revision().clone(),
            proposal.resulting_tree().clone(),
            protected_review_basis.evidence_commitment().clone(),
            source_execution_evidence,
            merge_nonce,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_parts(
        project: ProjectIdentity,
        proposal: ChangeProposalId,
        authority_epoch: Digest,
        project_policy: Digest,
        repository_policy_state: Digest,
        target_ref: RepositoryRef,
        expected_base: GitObjectId,
        proposed_revision: GitObjectId,
        resulting_tree: GitObjectId,
        protected_review_basis: Digest,
        source_execution_evidence: SourceExecutionEvidenceReferencesV1,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
    ) -> Result<Self, ProtectedMergeRequestError> {
        if expected_base == proposed_revision {
            return Err(ProtectedMergeRequestError::NoOpTransition);
        }
        let expected_algorithm = expected_base.algorithm();
        for (field, object) in [
            ("proposed_revision", &proposed_revision),
            ("resulting_tree", &resulting_tree),
        ] {
            if object.algorithm() != expected_algorithm {
                return Err(ProtectedMergeRequestError::MixedGitObjectAlgorithms {
                    field,
                    expected: expected_algorithm,
                    actual: object.algorithm(),
                });
            }
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            proposal,
            authority_epoch,
            project_policy,
            repository_policy_state,
            target_ref,
            expected_base,
            proposed_revision,
            resulting_tree,
            protected_review_basis,
            source_execution_evidence,
            merge_nonce,
        })
    }

    /// Project containing the protected transition.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable change proposal.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch already bound by proposal/review authority.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state already bound by proposal/review authority.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state for the protected transition.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact protected target ref.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Exact ref tip required before the merge attempt may consume the request.
    pub fn expected_base(&self) -> &GitObjectId {
        &self.expected_base
    }

    /// Exact proposed commit intended as the new ref tip.
    pub fn proposed_revision(&self) -> &GitObjectId {
        &self.proposed_revision
    }

    /// Exact tree claimed by the proposed commit.
    pub fn resulting_tree(&self) -> &GitObjectId {
        &self.resulting_tree
    }

    /// Exact FORGE-007N positive review-basis quorum evidence commitment.
    pub fn protected_review_basis(&self) -> &Digest {
        &self.protected_review_basis
    }

    /// Explicit source/execution evidence references awaiting positive rejoin.
    pub fn source_execution_evidence(&self) -> &SourceExecutionEvidenceReferencesV1 {
        &self.source_execution_evidence
    }

    /// Per-attempt nonce bound into this exact merge request.
    pub const fn merge_nonce(&self) -> &[u8; PROTECTED_MERGE_NONCE_LEN] {
        &self.merge_nonce
    }

    /// Canonical v1 request bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedMergeRequestError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(PROTECTED_MERGE_REQUEST_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.project_policy)?;
        push_digest(&mut out, &self.repository_policy_state)?;
        push_bytes(&mut out, "target_ref", self.target_ref.as_str().as_bytes())?;
        push_git_object(&mut out, &self.expected_base)?;
        push_git_object(&mut out, &self.proposed_revision)?;
        push_git_object(&mut out, &self.resulting_tree)?;
        push_digest(&mut out, &self.protected_review_basis)?;
        let evidence = self
            .source_execution_evidence
            .canonical_bytes()?;
        push_bytes(&mut out, "source_execution_evidence", &evidence)?;
        out.extend_from_slice(&self.merge_nonce);
        Ok(out)
    }

    /// Stable identity of this exact protected merge attempt.
    pub fn request_id(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<ProtectedMergeRequestId, ProtectedMergeRequestError> {
        Ok(ProtectedMergeRequestId(Digest::of_bytes(
            algorithm,
            &self.canonical_bytes()?,
        )))
    }
}

impl<'de> Deserialize<'de> for ProtectedMergeRequestV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireRequest {
            version: ProtocolVersion,
            project: ProjectIdentity,
            proposal: ChangeProposalId,
            authority_epoch: Digest,
            project_policy: Digest,
            repository_policy_state: Digest,
            target_ref: RepositoryRef,
            expected_base: GitObjectId,
            proposed_revision: GitObjectId,
            resulting_tree: GitObjectId,
            protected_review_basis: Digest,
            source_execution_evidence: SourceExecutionEvidenceReferencesV1,
            merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
        }

        let wire = WireRequest::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_parts(
            wire.project,
            wire.proposal,
            wire.authority_epoch,
            wire.project_policy,
            wire.repository_policy_state,
            wire.target_ref,
            wire.expected_base,
            wire.proposed_revision,
            wire.resulting_tree,
            wire.protected_review_basis,
            wire.source_execution_evidence,
            wire.merge_nonce,
        )
        .map_err(D::Error::custom)
    }
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ProtectedMergeRequestError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProtectedMergeRequestError::UnsupportedProtocolVersion(
            version.get(),
        ))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ProtectedMergeRequestError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), ProtectedMergeRequestError> {
    out.push(object.algorithm().code());
    push_bytes(out, "git_object", object.as_bytes())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ProtectedMergeRequestError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProtectedMergeRequestError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ProtectedMergeRequestError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Protected merge-request construction/canonicalization failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ProtectedMergeRequestError {
    /// Protected review basis belongs to another project.
    #[error("protected merge request project mismatch")]
    ProjectMismatch,
    /// Protected review basis belongs to another proposal.
    #[error("protected merge request proposal mismatch")]
    ProposalMismatch,
    /// Protected review basis uses another authority epoch.
    #[error("protected merge request authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Protected review basis uses another project-policy state.
    #[error("protected merge request project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Protected review basis uses another repository-policy state.
    #[error("protected merge request repository-policy mismatch")]
    RepositoryPolicyMismatch,
    /// Base and proposed revisions are identical.
    #[error("protected merge request is a no-op transition")]
    NoOpTransition,
    /// Git objects in one merge request use mixed object formats.
    #[error("{field} uses {actual:?}, expected {expected:?}")]
    MixedGitObjectAlgorithms {
        /// Field with the mismatched object algorithm.
        field: &'static str,
        /// Expected algorithm from the base revision.
        expected: GitObjectAlgorithm,
        /// Actual field algorithm.
        actual: GitObjectAlgorithm,
    },
    /// Unsupported protocol version.
    #[error("unsupported protected merge request protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Canonical field exceeded v1 bounds.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Actual length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Proposal identity/canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

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

    fn git_id(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn evidence(marker: u8) -> SourceExecutionEvidenceReferencesV1 {
        SourceExecutionEvidenceReferencesV1::new(
            ProposalSourceProfileV1::PinnedGitV1,
            OfflineExecutionProfileV1::M0OfflineEvidenceV6,
            digest(marker),
            digest(marker.wrapping_add(1)),
            digest(marker.wrapping_add(2)),
            digest(marker.wrapping_add(3)),
            digest(marker.wrapping_add(4)),
            digest(marker.wrapping_add(5)),
        )
    }

    fn request(
        nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
        protected_basis: Digest,
        evidence: SourceExecutionEvidenceReferencesV1,
    ) -> ProtectedMergeRequestV1 {
        ProtectedMergeRequestV1::from_parts(
            project(),
            ChangeProposalId::new(digest(0x20)),
            digest(0x21),
            digest(0x22),
            digest(0x23),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git_id(0x30),
            git_id(0x31),
            git_id(0x32),
            protected_basis,
            evidence,
            nonce,
        )
        .unwrap()
    }

    #[test]
    fn nonce_changes_exact_merge_request_identity() {
        let a = request([0x40; PROTECTED_MERGE_NONCE_LEN], digest(0x50), evidence(0x60));
        let b = request([0x41; PROTECTED_MERGE_NONCE_LEN], digest(0x50), evidence(0x60));
        assert_ne!(
            a.request_id(DigestAlgorithm::Sha256).unwrap(),
            b.request_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn protected_review_basis_changes_request_identity() {
        let a = request([0x40; PROTECTED_MERGE_NONCE_LEN], digest(0x50), evidence(0x60));
        let b = request([0x40; PROTECTED_MERGE_NONCE_LEN], digest(0x51), evidence(0x60));
        assert_ne!(
            a.request_id(DigestAlgorithm::Sha256).unwrap(),
            b.request_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn source_evidence_reference_changes_request_identity() {
        let a = request([0x40; PROTECTED_MERGE_NONCE_LEN], digest(0x50), evidence(0x60));
        let b = request([0x40; PROTECTED_MERGE_NONCE_LEN], digest(0x50), evidence(0x61));
        assert_ne!(
            a.request_id(DigestAlgorithm::Sha256).unwrap(),
            b.request_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn evidence_reference_order_is_fixed_by_schema() {
        let refs = evidence(0x60);
        let original = refs.digest(DigestAlgorithm::Sha256).unwrap();
        let changed = SourceExecutionEvidenceReferencesV1::new(
            refs.proposal_source_profile(),
            refs.offline_execution_profile(),
            refs.repository_request().clone(),
            refs.repository_verification().clone(),
            refs.source_state().clone(),
            refs.proposal_source().clone(),
            refs.offline_evidence().clone(),
            refs.execution_subject().clone(),
        );
        assert_ne!(original, changed.digest(DigestAlgorithm::Sha256).unwrap());
    }

    #[test]
    fn deserialized_request_remains_a_subject_not_authority() {
        let value = request([0x40; PROTECTED_MERGE_NONCE_LEN], digest(0x50), evidence(0x60));
        let encoded = serde_json::to_vec(&value).unwrap();
        let decoded: ProtectedMergeRequestV1 = serde_json::from_slice(&encoded).unwrap();
        assert_eq!(value, decoded);
    }

    #[test]
    fn mixed_git_object_algorithms_fail_closed() {
        let error = ProtectedMergeRequestV1::from_parts(
            project(),
            ChangeProposalId::new(digest(0x20)),
            digest(0x21),
            digest(0x22),
            digest(0x23),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git_id(0x30),
            GitObjectId::new(GitObjectAlgorithm::Sha256, vec![0x31; 32]).unwrap(),
            git_id(0x32),
            digest(0x50),
            evidence(0x60),
            [0x40; PROTECTED_MERGE_NONCE_LEN],
        )
        .unwrap_err();
        assert!(matches!(
            error,
            ProtectedMergeRequestError::MixedGitObjectAlgorithms {
                field: "proposed_revision",
                ..
            }
        ));
    }
}

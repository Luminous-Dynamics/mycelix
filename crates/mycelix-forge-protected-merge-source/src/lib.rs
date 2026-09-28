// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009B: current-line source qualification for protected merge requests.
//!
//! This crate deliberately reuses the useful FORGE-008A source theorem without
//! importing its obsolete collaboration ancestry. It joins one exact
//! [`ProtectedMergeRequestV1`] directly to the repository/source evidence that
//! request names.
//!
//! ```text
//! ProtectedMergeRequestV1
//! + exact ChangeProposal
//! + exact RepositoryVerificationRequest
//! + QualifiedRepositoryVerification
//! + independently verified Git source observation
//!     -> SourceQualifiedProtectedMergeRequestV1
//! ```
//!
//! The positive result closes only the repository request, repository
//! verification, proposal-source and source-state references. M0 OfflineEvidence
//! and hermetic execution-subject references remain explicitly unresolved for a
//! later theorem.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_protected_merge_request::{
    OfflineExecutionProfileV1, ProposalSourceProfileV1, ProtectedMergeRequestError,
    ProtectedMergeRequestId, ProtectedMergeRequestV1,
};
use mycelix_forge_repository::{
    GitObjectId, QualifiedRepositoryVerification, RepositoryAdoption, RepositoryPolicyState,
    RepositoryVerificationError, RepositoryVerificationRequest,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const SOURCE_OBSERVATION_DOMAIN_V1: &[u8] = b"mycelix-forge/proposal-source-observation/v1\0";
const REPOSITORY_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/proposal-repository-evidence/v1\0";
const QUALIFIED_SOURCE_DOMAIN_V1: &[u8] = b"mycelix-forge/qualified-proposal-source/v1\0";
const PROTECTED_MERGE_SOURCE_DOMAIN_V1: &[u8] =
    b"mycelix-forge/source-qualified-protected-merge-request/v1\0";
const MAX_ADAPTER_FIELD_LEN: usize = 128;

/// Stable identity of the concrete Git-source verifier used for one source
/// observation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SourceVerifierIdentityV1 {
    name: String,
    version: String,
}

impl SourceVerifierIdentityV1 {
    /// Construct a validated verifier identity.
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ProtectedMergeSourceError> {
        let name = name.into();
        let version = version.into();
        validate_adapter_field("name", &name)?;
        validate_adapter_field("version", &version)?;
        Ok(Self { name, version })
    }

    /// Verifier implementation/profile name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Verifier implementation/profile version.
    pub fn version(&self) -> &str {
        &self.version
    }
}

impl<'de> Deserialize<'de> for SourceVerifierIdentityV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireIdentity {
            name: String,
            version: String,
        }

        let wire = WireIdentity::deserialize(deserializer)?;
        Self::new(wire.name, wire.version).map_err(D::Error::custom)
    }
}

/// Raw observation of the exact Git objects named by one protected merge
/// request.
///
/// This remains non-authoritative until a concrete [`ProposalSourceVerifierV1`]
/// independently verifies it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProposalSourceObservationV1 {
    version: ProtocolVersion,
    adapter: SourceVerifierIdentityV1,
    proposal: ChangeProposalId,
    request_digest: Digest,
    source_state: Digest,
    observed_base: GitObjectId,
    observed_proposed: GitObjectId,
    observed_tree: GitObjectId,
    ancestry_evidence: Digest,
    commit_tree_evidence: Digest,
}

impl ProposalSourceObservationV1 {
    /// Construct a raw source observation.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        adapter: SourceVerifierIdentityV1,
        proposal: ChangeProposalId,
        request_digest: Digest,
        source_state: Digest,
        observed_base: GitObjectId,
        observed_proposed: GitObjectId,
        observed_tree: GitObjectId,
        ancestry_evidence: Digest,
        commit_tree_evidence: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            adapter,
            proposal,
            request_digest,
            source_state,
            observed_base,
            observed_proposed,
            observed_tree,
            ancestry_evidence,
            commit_tree_evidence,
        }
    }

    /// Concrete verifier identity that produced the observation.
    pub fn adapter(&self) -> &SourceVerifierIdentityV1 {
        &self.adapter
    }

    /// Exact immutable proposal observed.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact repository request observed.
    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    /// Exact object-store/bundle/source-state commitment inspected.
    pub fn source_state(&self) -> &Digest {
        &self.source_state
    }

    /// Base commit observed.
    pub fn observed_base(&self) -> &GitObjectId {
        &self.observed_base
    }

    /// Proposed commit observed.
    pub fn observed_proposed(&self) -> &GitObjectId {
        &self.observed_proposed
    }

    /// Tree referenced by the proposed commit.
    pub fn observed_tree(&self) -> &GitObjectId {
        &self.observed_tree
    }

    /// Commitment to ancestry/reachability evidence.
    pub fn ancestry_evidence(&self) -> &Digest {
        &self.ancestry_evidence
    }

    /// Commitment to commit/type/tree evidence.
    pub fn commit_tree_evidence(&self) -> &Digest {
        &self.commit_tree_evidence
    }

    /// Canonical v1 observation bytes. This intentionally preserves the
    /// FORGE-008A domain so a concrete verifier port can remain byte-compatible.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedMergeSourceError> {
        validate_adapter_field("name", self.adapter.name())?;
        validate_adapter_field("version", self.adapter.version())?;

        let mut out = Vec::new();
        out.extend_from_slice(SOURCE_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_bytes(&mut out, "adapter_name", self.adapter.name().as_bytes())?;
        push_bytes(
            &mut out,
            "adapter_version",
            self.adapter.version().as_bytes(),
        )?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.request_digest)?;
        push_digest(&mut out, &self.source_state)?;
        push_git_object(&mut out, &self.observed_base)?;
        push_git_object(&mut out, &self.observed_proposed)?;
        push_git_object(&mut out, &self.observed_tree)?;
        push_digest(&mut out, &self.ancestry_evidence)?;
        push_digest(&mut out, &self.commit_tree_evidence)?;
        Ok(out)
    }

    /// Stable observation commitment.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedMergeSourceError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ProposalSourceObservationV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireObservation {
            version: ProtocolVersion,
            adapter: SourceVerifierIdentityV1,
            proposal: ChangeProposalId,
            request_digest: Digest,
            source_state: Digest,
            observed_base: GitObjectId,
            observed_proposed: GitObjectId,
            observed_tree: GitObjectId,
            ancestry_evidence: Digest,
            commit_tree_evidence: Digest,
        }

        let wire = WireObservation::deserialize(deserializer)?;
        if wire.version != ProtocolVersion::CURRENT {
            return Err(D::Error::custom("unsupported proposal-source protocol version"));
        }
        Ok(Self::new(
            wire.adapter,
            wire.proposal,
            wire.request_digest,
            wire.source_state,
            wire.observed_base,
            wire.observed_proposed,
            wire.observed_tree,
            wire.ancestry_evidence,
            wire.commit_tree_evidence,
        ))
    }
}

/// Concrete source-verifier rejection.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SourceVerifierErrorV1 {
    /// Git source/object verification rejected the observation.
    #[error("proposal source verifier rejected evidence")]
    Rejected,
}

/// Concrete verifier for Git object/type/tree/ancestry semantics.
pub trait ProposalSourceVerifierV1 {
    /// Stable verifier identity/profile.
    fn identity(&self) -> SourceVerifierIdentityV1;

    /// Independently verify the source observation and return verifier-side
    /// evidence on success.
    fn verify_source(
        &self,
        proposal: &ChangeProposal,
        request: &RepositoryVerificationRequest,
        observation: &ProposalSourceObservationV1,
    ) -> Result<Digest, SourceVerifierErrorV1>;
}

/// Positive result proving the source-side references of one exact protected
/// merge request have been rejoined to actual positive repository/source
/// evidence.
///
/// This type intentionally has no `Deserialize` implementation. Consumers must
/// rerun [`qualify_protected_merge_source_v1`] from live positive inputs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SourceQualifiedProtectedMergeRequestV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    protected_merge_request: ProtectedMergeRequestId,
    repository_request: Digest,
    repository_verification: Digest,
    proposal_source: Digest,
    source_state: Digest,
    source_verifier_evidence: Digest,
    pending_offline_evidence: Digest,
    pending_execution_subject: Digest,
    evidence_commitment: Digest,
}

impl SourceQualifiedProtectedMergeRequestV1 {
    /// Project containing the transition.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected merge request whose source references were qualified.
    pub fn protected_merge_request(&self) -> &ProtectedMergeRequestId {
        &self.protected_merge_request
    }

    /// Rejoined repository request digest.
    pub fn repository_request(&self) -> &Digest {
        &self.repository_request
    }

    /// Rejoined positive repository-verification commitment.
    pub fn repository_verification(&self) -> &Digest {
        &self.repository_verification
    }

    /// Rejoined proposal-source qualification commitment.
    pub fn proposal_source(&self) -> &Digest {
        &self.proposal_source
    }

    /// Rejoined source-state commitment.
    pub fn source_state(&self) -> &Digest {
        &self.source_state
    }

    /// Concrete source-verifier evidence.
    pub fn source_verifier_evidence(&self) -> &Digest {
        &self.source_verifier_evidence
    }

    /// M0 OfflineEvidence reference still awaiting a positive M0 rejoin.
    pub fn pending_offline_evidence(&self) -> &Digest {
        &self.pending_offline_evidence
    }

    /// Hermetic execution-subject reference still awaiting a positive M0 rejoin.
    pub fn pending_execution_subject(&self) -> &Digest {
        &self.pending_execution_subject
    }

    /// Aggregate source-qualified protected-merge evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Rejoin the source side of one exact protected merge request.
///
/// Four request references are closed here:
///
/// - repository request;
/// - repository verification;
/// - proposal-source evidence;
/// - source-state evidence.
///
/// M0 OfflineEvidence and the hermetic execution subject are preserved as
/// unresolved references for the next theorem.
#[allow(clippy::too_many_arguments)]
pub fn qualify_protected_merge_source_v1<V: ProposalSourceVerifierV1>(
    protected_merge_request: &ProtectedMergeRequestV1,
    proposal: &ChangeProposal,
    adoption: &RepositoryAdoption,
    repository_policy_state: &RepositoryPolicyState,
    repository_request: &RepositoryVerificationRequest,
    repository_verification: &QualifiedRepositoryVerification,
    observation: ProposalSourceObservationV1,
    verifier: &V,
) -> Result<SourceQualifiedProtectedMergeRequestV1, ProtectedMergeSourceError> {
    rejoin_request_to_proposal(protected_merge_request, proposal)?;

    if protected_merge_request
        .source_execution_evidence()
        .proposal_source_profile()
        != ProposalSourceProfileV1::PinnedGitV1
    {
        return Err(ProtectedMergeSourceError::UnexpectedProposalSourceProfile);
    }
    if protected_merge_request
        .source_execution_evidence()
        .offline_execution_profile()
        != OfflineExecutionProfileV1::M0OfflineEvidenceV6
    {
        return Err(ProtectedMergeSourceError::UnexpectedOfflineExecutionProfile);
    }

    if proposal.project() != adoption.project()
        || proposal.project() != repository_request.project()
    {
        return Err(ProtectedMergeSourceError::ProjectMismatch);
    }
    if proposal.target_ref() != repository_request.reference() {
        return Err(ProtectedMergeSourceError::TargetRefMismatch);
    }
    if proposal.repository_policy_state() != repository_request.repository_policy_state() {
        return Err(ProtectedMergeSourceError::RepositoryPolicyMismatch);
    }

    let request_algorithm = protected_merge_request
        .source_execution_evidence()
        .repository_request()
        .algorithm();
    let expected_repository_request = RepositoryVerificationRequest::new(
        adoption,
        proposal.base_revision().clone(),
        proposal.proposed_revision().clone(),
        proposal.authority_epoch().clone(),
        proposal.project_policy().clone(),
        repository_policy_state,
        request_algorithm,
    )?;
    if expected_repository_request.canonical_bytes()? != repository_request.canonical_bytes()? {
        return Err(ProtectedMergeSourceError::RepositoryRequestMismatch);
    }

    let repository_request_digest = repository_request.digest(request_algorithm)?;
    if protected_merge_request
        .source_execution_evidence()
        .repository_request()
        != &repository_request_digest
    {
        return Err(ProtectedMergeSourceError::RepositoryRequestReferenceMismatch);
    }

    let structural = repository_verification.structural();
    let positive_request_digest = repository_request.digest(structural.request_digest().algorithm())?;
    if structural.request_digest() != &positive_request_digest {
        return Err(ProtectedMergeSourceError::RepositoryVerificationRequestMismatch);
    }
    if structural.observed_tip() != proposal.proposed_revision() {
        return Err(ProtectedMergeSourceError::RepositoryTipMismatch);
    }
    if structural.repository_policy_state() != proposal.repository_policy_state() {
        return Err(ProtectedMergeSourceError::RepositoryPolicyMismatch);
    }

    if observation.adapter() != &verifier.identity() {
        return Err(ProtectedMergeSourceError::VerifierIdentityMismatch);
    }
    let expected_proposal = proposal.proposal_id(observation.proposal().commitment().algorithm())?;
    if observation.proposal() != &expected_proposal {
        return Err(ProtectedMergeSourceError::ProposalObservationMismatch);
    }
    let observation_request = repository_request.digest(observation.request_digest().algorithm())?;
    if observation.request_digest() != &observation_request {
        return Err(ProtectedMergeSourceError::ObservationRequestMismatch);
    }
    if observation.observed_base() != proposal.base_revision() {
        return Err(ProtectedMergeSourceError::ObservedBaseMismatch);
    }
    if observation.observed_proposed() != proposal.proposed_revision() {
        return Err(ProtectedMergeSourceError::ObservedProposedMismatch);
    }
    if observation.observed_tree() != proposal.resulting_tree() {
        return Err(ProtectedMergeSourceError::ObservedTreeMismatch);
    }

    let verifier_evidence = verifier.verify_source(proposal, repository_request, &observation)?;
    let evidence_algorithm = verifier_evidence.algorithm();
    let repository_evidence =
        repository_verification_commitment_v1(repository_verification, evidence_algorithm)?;
    let proposal_source = proposal_source_commitment_v1(
        proposal,
        repository_request,
        &repository_evidence,
        &observation,
        &verifier_evidence,
        evidence_algorithm,
    )?;

    let references = protected_merge_request.source_execution_evidence();
    if references.repository_verification() != &repository_evidence {
        return Err(ProtectedMergeSourceError::RepositoryVerificationReferenceMismatch);
    }
    if references.proposal_source() != &proposal_source {
        return Err(ProtectedMergeSourceError::ProposalSourceReferenceMismatch);
    }
    if references.source_state() != observation.source_state() {
        return Err(ProtectedMergeSourceError::SourceStateReferenceMismatch);
    }

    let protected_merge_request_id = protected_merge_request.request_id(evidence_algorithm)?;
    let proposal_id = proposal.proposal_id(evidence_algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(PROTECTED_MERGE_SOURCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, proposal_id.commitment())?;
    push_digest(&mut out, protected_merge_request_id.commitment())?;
    push_digest(&mut out, &repository_request_digest)?;
    push_digest(&mut out, &repository_evidence)?;
    push_digest(&mut out, &proposal_source)?;
    push_digest(&mut out, observation.source_state())?;
    push_digest(&mut out, &verifier_evidence)?;
    push_digest(&mut out, references.offline_evidence())?;
    push_digest(&mut out, references.execution_subject())?;
    let evidence_commitment = Digest::of_bytes(evidence_algorithm, &out);

    Ok(SourceQualifiedProtectedMergeRequestV1 {
        project: proposal.project().clone(),
        proposal: proposal_id,
        protected_merge_request: protected_merge_request_id,
        repository_request: repository_request_digest,
        repository_verification: repository_evidence,
        proposal_source,
        source_state: observation.source_state().clone(),
        source_verifier_evidence: verifier_evidence,
        pending_offline_evidence: references.offline_evidence().clone(),
        pending_execution_subject: references.execution_subject().clone(),
        evidence_commitment,
    })
}

fn rejoin_request_to_proposal(
    request: &ProtectedMergeRequestV1,
    proposal: &ChangeProposal,
) -> Result<(), ProtectedMergeSourceError> {
    if request.project() != proposal.project() {
        return Err(ProtectedMergeSourceError::ProjectMismatch);
    }
    let proposal_id = proposal.proposal_id(request.proposal().commitment().algorithm())?;
    if request.proposal() != &proposal_id {
        return Err(ProtectedMergeSourceError::ProposalRequestMismatch);
    }
    if request.authority_epoch() != proposal.authority_epoch() {
        return Err(ProtectedMergeSourceError::AuthorityEpochMismatch);
    }
    if request.project_policy() != proposal.project_policy() {
        return Err(ProtectedMergeSourceError::ProjectPolicyMismatch);
    }
    if request.repository_policy_state() != proposal.repository_policy_state() {
        return Err(ProtectedMergeSourceError::RepositoryPolicyMismatch);
    }
    if request.target_ref() != proposal.target_ref() {
        return Err(ProtectedMergeSourceError::TargetRefMismatch);
    }
    if request.expected_base() != proposal.base_revision() {
        return Err(ProtectedMergeSourceError::RequestBaseMismatch);
    }
    if request.proposed_revision() != proposal.proposed_revision() {
        return Err(ProtectedMergeSourceError::RequestProposedMismatch);
    }
    if request.resulting_tree() != proposal.resulting_tree() {
        return Err(ProtectedMergeSourceError::RequestTreeMismatch);
    }
    Ok(())
}

fn proposal_source_commitment_v1(
    proposal: &ChangeProposal,
    request: &RepositoryVerificationRequest,
    repository_evidence: &Digest,
    observation: &ProposalSourceObservationV1,
    verifier_evidence: &Digest,
    algorithm: DigestAlgorithm,
) -> Result<Digest, ProtectedMergeSourceError> {
    let proposal_id = proposal.proposal_id(algorithm)?;
    let request_digest = request.digest(algorithm)?;
    let observation_digest = observation.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(QUALIFIED_SOURCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, proposal_id.commitment())?;
    push_digest(&mut out, &request_digest)?;
    push_digest(&mut out, repository_evidence)?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, verifier_evidence)?;
    Ok(Digest::of_bytes(algorithm, &out))
}

fn repository_verification_commitment_v1(
    qualified: &QualifiedRepositoryVerification,
    algorithm: DigestAlgorithm,
) -> Result<Digest, ProtectedMergeSourceError> {
    let structural = qualified.structural();
    let mut out = Vec::new();
    out.extend_from_slice(REPOSITORY_EVIDENCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, structural.request_digest())?;
    push_bytes(
        &mut out,
        "repository_adapter_name",
        structural.adapter().name().as_bytes(),
    )?;
    push_bytes(
        &mut out,
        "repository_adapter_version",
        structural.adapter().version().as_bytes(),
    )?;
    push_git_object(&mut out, structural.observed_tip())?;
    push_digest(&mut out, structural.repository_policy_state())?;
    push_optional_digest(&mut out, structural.history_commitment())?;
    push_optional_digest(&mut out, structural.evidence_commitment())?;
    push_optional_digest(&mut out, qualified.policy_lineage_commitment())?;
    let count = u16::try_from(structural.capabilities().len()).map_err(|_| {
        ProtectedMergeSourceError::CanonicalFieldTooLarge {
            field: "repository_capabilities",
            len: structural.capabilities().len(),
            max: u16::MAX as usize,
        }
    })?;
    out.extend_from_slice(&count.to_be_bytes());
    for capability in structural.capabilities() {
        out.extend_from_slice(&capability.code().to_be_bytes());
    }
    Ok(Digest::of_bytes(algorithm, &out))
}

fn validate_adapter_field(
    field: &'static str,
    value: &str,
) -> Result<(), ProtectedMergeSourceError> {
    let len = value.len();
    if len == 0 || len > MAX_ADAPTER_FIELD_LEN {
        return Err(ProtectedMergeSourceError::InvalidVerifierField { field, len });
    }
    Ok(())
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ProtectedMergeSourceError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), ProtectedMergeSourceError> {
    out.push(object.algorithm().code());
    push_bytes(out, "git_object", object.as_bytes())
}

fn push_optional_digest(
    out: &mut Vec<u8>,
    digest: Option<&Digest>,
) -> Result<(), ProtectedMergeSourceError> {
    match digest {
        Some(digest) => {
            out.push(1);
            push_digest(out, digest)
        }
        None => {
            out.push(0);
            Ok(())
        }
    }
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ProtectedMergeSourceError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProtectedMergeSourceError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ProtectedMergeSourceError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// FORGE-009B source-qualification failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ProtectedMergeSourceError {
    /// Proposal/request/adoption project mismatch.
    #[error("protected merge source project mismatch")]
    ProjectMismatch,
    /// Protected merge request names another proposal.
    #[error("protected merge request does not name the supplied proposal")]
    ProposalRequestMismatch,
    /// Authority epoch differs between request and proposal.
    #[error("protected merge source authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Project policy differs between request and proposal.
    #[error("protected merge source project policy mismatch")]
    ProjectPolicyMismatch,
    /// Repository policy differs between supplied evidence objects.
    #[error("protected merge source repository policy mismatch")]
    RepositoryPolicyMismatch,
    /// Protected target ref differs.
    #[error("protected merge source target ref mismatch")]
    TargetRefMismatch,
    /// Protected merge request names another base commit.
    #[error("protected merge request base commit mismatch")]
    RequestBaseMismatch,
    /// Protected merge request names another proposed commit.
    #[error("protected merge request proposed commit mismatch")]
    RequestProposedMismatch,
    /// Protected merge request names another resulting tree.
    #[error("protected merge request resulting tree mismatch")]
    RequestTreeMismatch,
    /// Request does not require the pinned Git source profile.
    #[error("protected merge request has unexpected proposal-source profile")]
    UnexpectedProposalSourceProfile,
    /// Request does not require frozen M0 v6 for the later execution join.
    #[error("protected merge request has unexpected offline execution profile")]
    UnexpectedOfflineExecutionProfile,
    /// Reconstructed repository request differs from the supplied request.
    #[error("proposal does not reconstruct the exact repository verification request")]
    RepositoryRequestMismatch,
    /// Request's repository-request reference does not match live evidence.
    #[error("protected merge request repository-request reference mismatch")]
    RepositoryRequestReferenceMismatch,
    /// Positive repository verification belongs to another request.
    #[error("qualified repository verification belongs to another request")]
    RepositoryVerificationRequestMismatch,
    /// Positive repository verification observed another proposed tip.
    #[error("qualified repository verification observed another tip")]
    RepositoryTipMismatch,
    /// Source observation names another verifier implementation/profile.
    #[error("proposal source verifier identity mismatch")]
    VerifierIdentityMismatch,
    /// Source observation names another proposal.
    #[error("proposal source observation names another proposal")]
    ProposalObservationMismatch,
    /// Source observation names another repository request.
    #[error("proposal source observation names another repository request")]
    ObservationRequestMismatch,
    /// Source observation saw another base commit.
    #[error("proposal source observation base mismatch")]
    ObservedBaseMismatch,
    /// Source observation saw another proposed commit.
    #[error("proposal source observation proposed commit mismatch")]
    ObservedProposedMismatch,
    /// Source observation saw another resulting tree.
    #[error("proposal source observation tree mismatch")]
    ObservedTreeMismatch,
    /// Protected merge request's repository-verification reference is wrong.
    #[error("protected merge request repository-verification reference mismatch")]
    RepositoryVerificationReferenceMismatch,
    /// Protected merge request's proposal-source reference is wrong.
    #[error("protected merge request proposal-source reference mismatch")]
    ProposalSourceReferenceMismatch,
    /// Protected merge request's source-state reference is wrong.
    #[error("protected merge request source-state reference mismatch")]
    SourceStateReferenceMismatch,
    /// Verifier identity field is invalid.
    #[error("invalid source verifier field {field}: length {len}")]
    InvalidVerifierField {
        /// Field name.
        field: &'static str,
        /// Actual length.
        len: usize,
    },
    /// Canonical field exceeded protocol encoding bounds.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Actual length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Proposal identity/canonicalization failed.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Repository request/canonicalization failed.
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    /// Protected merge-request identity failed.
    #[error(transparent)]
    ProtectedMergeRequest(#[from] ProtectedMergeRequestError),
    /// Concrete source verifier rejected the observation.
    #[error(transparent)]
    Verifier(#[from] SourceVerifierErrorV1),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authority::{
        AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant, PrincipalId,
    };
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1, TrustedProviderVerifierV1,
    };
    use mycelix_forge_protected_merge_request::SourceExecutionEvidenceReferencesV1;
    use mycelix_forge_repository::{
        qualify_evidence_backed_observation, AdapterIdentity, AdapterObservation, AdapterOutcome,
        EvidenceBackedAdapterObservation, GitObjectAlgorithm, RepositoryRef, RepositoryTip,
        VerificationCapability, VerificationProfile,
    };
    use serde::Serialize;

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

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn authority(project: &ProjectIdentity, principal: &PrincipalId) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project: project.clone(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: None,
            grants: vec![PrincipalGrant::new(principal.clone(), [Capability::ManageAuthority]).unwrap()],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn project_policy(project: &ProjectIdentity) -> ProjectPolicyStateV1 {
        let trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![TrustedProviderVerifierV1::new(digest(0x20), digest(0x21))],
        )
        .unwrap();
        ProjectPolicyStateV1::new(
            project.clone(),
            0,
            None,
            &trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    struct AcceptVerifier;

    impl ProposalSourceVerifierV1 for AcceptVerifier {
        fn identity(&self) -> SourceVerifierIdentityV1 {
            SourceVerifierIdentityV1::new("pinned-git", "1").unwrap()
        }

        fn verify_source(
            &self,
            _proposal: &ChangeProposal,
            _request: &RepositoryVerificationRequest,
            observation: &ProposalSourceObservationV1,
        ) -> Result<Digest, SourceVerifierErrorV1> {
            let mut bytes = Vec::new();
            bytes.extend_from_slice(b"test/source-verifier/v1\0");
            bytes.extend_from_slice(observation.ancestry_evidence().as_bytes());
            bytes.extend_from_slice(observation.commit_tree_evidence().as_bytes());
            Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
        }
    }

    struct Fixture {
        proposal: ChangeProposal,
        adoption: RepositoryAdoption,
        repository_policy: RepositoryPolicyState,
        repository_request: RepositoryVerificationRequest,
        repository_verification: QualifiedRepositoryVerification,
        observation: ProposalSourceObservationV1,
        repository_evidence: Digest,
        proposal_source: Digest,
    }

    fn fixture() -> Fixture {
        let project = project();
        let proposer = PrincipalId::new(digest(0x30));
        let authority = authority(&project, &proposer);
        let project_policy = project_policy(&project);
        let repository_policy =
            RepositoryPolicyState::new(project.clone(), 0, None, digest(0x40)).unwrap();
        let target_ref = RepositoryRef::new("refs/heads/main").unwrap();
        let base = git(0x50);
        let proposed = git(0x51);
        let tree = git(0x52);
        let proposal = ChangeProposal::new(
            proposer,
            &authority,
            &project_policy,
            &repository_policy,
            target_ref.clone(),
            base.clone(),
            proposed.clone(),
            tree.clone(),
            digest(0x53),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let authority_digest = authority.digest(DigestAlgorithm::Sha256).unwrap();
        let project_policy_digest = project_policy.digest(DigestAlgorithm::Sha256).unwrap();
        let adoption = RepositoryAdoption::new(
            project,
            RepositoryTip::new(target_ref, base.clone()),
            authority_digest.clone(),
            project_policy_digest.clone(),
            repository_policy.policy_digest().clone(),
            900,
        );
        let repository_request = RepositoryVerificationRequest::new(
            &adoption,
            base.clone(),
            proposed.clone(),
            authority_digest,
            project_policy_digest,
            &repository_policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let request_digest = repository_request.digest(DigestAlgorithm::Sha256).unwrap();
        let observation = AdapterObservation::new(
            AdapterIdentity::new("m0-test", "1").unwrap(),
            request_digest.clone(),
            proposed.clone(),
            repository_policy.digest(DigestAlgorithm::Sha256).unwrap(),
            Some(digest(0x60)),
            Some(digest(0x61)),
            [
                VerificationCapability::RefTipBinding,
                VerificationCapability::FullHistory,
                VerificationCapability::ProtectedRewriteDetection,
                VerificationCapability::PolicyLineageMonotonic,
                VerificationCapability::OfflineEvidence,
            ],
            AdapterOutcome::Verified,
        )
        .unwrap();
        let repository_verification = qualify_evidence_backed_observation(
            &VerificationProfile::m0_protected_source(),
            &repository_request,
            EvidenceBackedAdapterObservation::new(observation, Some(digest(0x62))).unwrap(),
        )
        .unwrap();
        let proposal_id = proposal.proposal_id(DigestAlgorithm::Sha256).unwrap();
        let source_observation = ProposalSourceObservationV1::new(
            AcceptVerifier.identity(),
            proposal_id,
            request_digest,
            digest(0x63),
            base,
            proposed,
            tree,
            digest(0x64),
            digest(0x65),
        );
        let verifier_evidence = AcceptVerifier
            .verify_source(&proposal, &repository_request, &source_observation)
            .unwrap();
        let repository_evidence = repository_verification_commitment_v1(
            &repository_verification,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let proposal_source = proposal_source_commitment_v1(
            &proposal,
            &repository_request,
            &repository_evidence,
            &source_observation,
            &verifier_evidence,
            DigestAlgorithm::Sha256,
        )
        .unwrap();

        Fixture {
            proposal,
            adoption,
            repository_policy,
            repository_request,
            repository_verification,
            observation: source_observation,
            repository_evidence,
            proposal_source,
        }
    }

    #[derive(Serialize)]
    struct WireProtectedMergeRequest {
        version: ProtocolVersion,
        project: ProjectIdentity,
        proposal: ChangeProposalId,
        authority_epoch: Digest,
        project_policy: Digest,
        repository_policy_state: Digest,
        target_ref: mycelix_forge_repository::RepositoryRef,
        expected_base: GitObjectId,
        proposed_revision: GitObjectId,
        resulting_tree: GitObjectId,
        protected_review_basis: Digest,
        source_execution_evidence: SourceExecutionEvidenceReferencesV1,
        merge_nonce: [u8; 32],
    }

    fn protected_request(
        fixture: &Fixture,
        repository_evidence: Digest,
        proposal_source: Digest,
        source_state: Digest,
        offline_evidence: Digest,
        execution_subject: Digest,
    ) -> ProtectedMergeRequestV1 {
        let proposal = &fixture.proposal;
        let refs = SourceExecutionEvidenceReferencesV1::new(
            ProposalSourceProfileV1::PinnedGitV1,
            OfflineExecutionProfileV1::M0OfflineEvidenceV6,
            fixture
                .repository_request
                .digest(DigestAlgorithm::Sha256)
                .unwrap(),
            repository_evidence,
            proposal_source,
            source_state,
            offline_evidence,
            execution_subject,
        );
        let wire = WireProtectedMergeRequest {
            version: ProtocolVersion::CURRENT,
            project: proposal.project().clone(),
            proposal: proposal.proposal_id(DigestAlgorithm::Sha256).unwrap(),
            authority_epoch: proposal.authority_epoch().clone(),
            project_policy: proposal.project_policy().clone(),
            repository_policy_state: proposal.repository_policy_state().clone(),
            target_ref: proposal.target_ref().clone(),
            expected_base: proposal.base_revision().clone(),
            proposed_revision: proposal.proposed_revision().clone(),
            resulting_tree: proposal.resulting_tree().clone(),
            protected_review_basis: digest(0x70),
            source_execution_evidence: refs,
            merge_nonce: [0x71; 32],
        };
        serde_json::from_value(serde_json::to_value(wire).unwrap()).unwrap()
    }

    #[test]
    fn exact_source_references_upgrade_only_the_exact_request() {
        let f = fixture();
        let request = protected_request(
            &f,
            f.repository_evidence.clone(),
            f.proposal_source.clone(),
            f.observation.source_state().clone(),
            digest(0x80),
            digest(0x81),
        );
        let qualified = qualify_protected_merge_source_v1(
            &request,
            &f.proposal,
            &f.adoption,
            &f.repository_policy,
            &f.repository_request,
            &f.repository_verification,
            f.observation.clone(),
            &AcceptVerifier,
        )
        .unwrap();

        assert_eq!(qualified.repository_verification(), &f.repository_evidence);
        assert_eq!(qualified.proposal_source(), &f.proposal_source);
        assert_eq!(qualified.source_state(), f.observation.source_state());
        assert_eq!(qualified.pending_offline_evidence(), &digest(0x80));
        assert_eq!(qualified.pending_execution_subject(), &digest(0x81));
    }

    #[test]
    fn proposal_source_reference_cannot_be_borrowed_or_substituted() {
        let f = fixture();
        let request = protected_request(
            &f,
            f.repository_evidence.clone(),
            digest(0x91),
            f.observation.source_state().clone(),
            digest(0x80),
            digest(0x81),
        );
        let error = qualify_protected_merge_source_v1(
            &request,
            &f.proposal,
            &f.adoption,
            &f.repository_policy,
            &f.repository_request,
            &f.repository_verification,
            f.observation.clone(),
            &AcceptVerifier,
        )
        .unwrap_err();
        assert_eq!(error, ProtectedMergeSourceError::ProposalSourceReferenceMismatch);
    }

    #[test]
    fn source_state_reference_must_match_the_verified_observation() {
        let f = fixture();
        let request = protected_request(
            &f,
            f.repository_evidence.clone(),
            f.proposal_source.clone(),
            digest(0x92),
            digest(0x80),
            digest(0x81),
        );
        let error = qualify_protected_merge_source_v1(
            &request,
            &f.proposal,
            &f.adoption,
            &f.repository_policy,
            &f.repository_request,
            &f.repository_verification,
            f.observation.clone(),
            &AcceptVerifier,
        )
        .unwrap_err();
        assert_eq!(error, ProtectedMergeSourceError::SourceStateReferenceMismatch);
    }

    #[test]
    fn offline_evidence_remains_deliberately_unqualified_at_this_layer() {
        let f = fixture();
        let request = protected_request(
            &f,
            f.repository_evidence.clone(),
            f.proposal_source.clone(),
            f.observation.source_state().clone(),
            digest(0xa0),
            digest(0xa1),
        );
        let qualified = qualify_protected_merge_source_v1(
            &request,
            &f.proposal,
            &f.adoption,
            &f.repository_policy,
            &f.repository_request,
            &f.repository_verification,
            f.observation.clone(),
            &AcceptVerifier,
        )
        .unwrap();
        assert_eq!(qualified.pending_offline_evidence(), &digest(0xa0));
        assert_eq!(qualified.pending_execution_subject(), &digest(0xa1));
    }
}

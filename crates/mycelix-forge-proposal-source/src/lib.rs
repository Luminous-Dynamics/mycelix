// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bind exact Forge proposals to protected repository verification and Git object evidence.
//!
//! FORGE-006 intentionally makes `base_revision`, `proposed_revision`, and
//! `resulting_tree` immutable claims. This crate is the evidence layer that
//! binds those claims to the exact protected-source request already qualified
//! by the repository layer, plus an independent Git-source verifier for
//! ancestry/object/tree facts.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_repository::{
    GitObjectId, QualifiedRepositoryVerification, RepositoryAdoption, RepositoryPolicyState,
    RepositoryVerificationError, RepositoryVerificationRequest, VerificationCapability,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const SOURCE_OBSERVATION_DOMAIN_V1: &[u8] = b"mycelix-forge/proposal-source-observation/v1\0";
const REPOSITORY_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/proposal-repository-evidence/v1\0";
const QUALIFIED_SOURCE_DOMAIN_V1: &[u8] = b"mycelix-forge/qualified-proposal-source/v1\0";
const MAX_ADAPTER_FIELD_LEN: usize = 128;

/// Stable identity of a Git-source verifier implementation/profile.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SourceVerifierIdentity {
    name: String,
    version: String,
}

impl SourceVerifierIdentity {
    /// Construct a validated v1 verifier identity.
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ProposalSourceError> {
        let name = name.into();
        let version = version.into();
        validate_adapter_field("name", &name)?;
        validate_adapter_field("version", &version)?;
        Ok(Self { name, version })
    }

    /// Implementation/profile name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Implementation/profile version.
    pub fn version(&self) -> &str {
        &self.version
    }
}

impl<'de> Deserialize<'de> for SourceVerifierIdentity {
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

/// Raw Git-source observation over one exact proposal/request pair.
///
/// This is not self-authenticating evidence. A concrete [`ProposalSourceVerifier`]
/// must independently verify the named source state and return positive
/// verifier evidence before Forge constructs [`QualifiedProposalSource`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProposalSourceObservation {
    version: ProtocolVersion,
    adapter: SourceVerifierIdentity,
    proposal: ChangeProposalId,
    request_digest: Digest,
    source_state: Digest,
    observed_base: GitObjectId,
    observed_proposed: GitObjectId,
    observed_tree: GitObjectId,
    ancestry_evidence: Digest,
    commit_tree_evidence: Digest,
}

impl ProposalSourceObservation {
    /// Construct a raw source observation.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        adapter: SourceVerifierIdentity,
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

    /// Verifier implementation/profile claiming these facts.
    pub fn adapter(&self) -> &SourceVerifierIdentity {
        &self.adapter
    }

    /// Exact immutable proposal claimed to have been inspected.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected-repository request claimed to have been inspected.
    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    /// Exact object-store/bundle/source-state root inspected by the verifier.
    pub fn source_state(&self) -> &Digest {
        &self.source_state
    }

    /// Base revision observed by the verifier.
    pub fn observed_base(&self) -> &GitObjectId {
        &self.observed_base
    }

    /// Proposed revision observed by the verifier.
    pub fn observed_proposed(&self) -> &GitObjectId {
        &self.observed_proposed
    }

    /// Tree referenced by the proposed commit according to the verifier.
    pub fn observed_tree(&self) -> &GitObjectId {
        &self.observed_tree
    }

    /// Commitment to ancestry/reachability evidence.
    pub fn ancestry_evidence(&self) -> &Digest {
        &self.ancestry_evidence
    }

    /// Commitment to commit-object/type/tree evidence.
    pub fn commit_tree_evidence(&self) -> &Digest {
        &self.commit_tree_evidence
    }

    /// Canonical v1 bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProposalSourceError> {
        validate_adapter_field("name", self.adapter.name())?;
        validate_adapter_field("version", self.adapter.version())?;
        let mut out = Vec::new();
        out.extend_from_slice(SOURCE_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_bytes(&mut out, "adapter_name", self.adapter.name().as_bytes())?;
        push_bytes(&mut out, "adapter_version", self.adapter.version().as_bytes())?;
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

    /// Exact raw observation commitment.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ProposalSourceError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ProposalSourceObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireObservation {
            version: ProtocolVersion,
            adapter: SourceVerifierIdentity,
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

/// Adapter-specific verification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SourceVerifierError {
    /// Source verifier rejected ancestry/object/tree evidence.
    #[error("proposal source verifier rejected evidence")]
    Rejected,
}

/// Concrete verifier for Git source/object semantics.
pub trait ProposalSourceVerifier {
    /// Stable verifier identity/profile.
    fn identity(&self) -> SourceVerifierIdentity;

    /// Independently verify ancestry/object/tree facts and return a verifier
    /// evidence commitment on success.
    fn verify_source(
        &self,
        proposal: &ChangeProposal,
        request: &RepositoryVerificationRequest,
        observation: &ProposalSourceObservation,
    ) -> Result<Digest, SourceVerifierError>;
}

/// Positive result binding proposal claims to protected-repository evidence and
/// independently verified Git source/object facts.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedProposalSource {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    request_digest: Digest,
    repository_evidence: Digest,
    source_observation: ProposalSourceObservation,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

impl QualifiedProposalSource {
    /// Project containing the proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal whose source claims were verified.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected repository request bound to this proposal.
    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    /// Commitment to the positive repository verification consumed here.
    pub fn repository_evidence(&self) -> &Digest {
        &self.repository_evidence
    }

    /// Raw Git-source observation independently verified by the adapter.
    pub fn source_observation(&self) -> &ProposalSourceObservation {
        &self.source_observation
    }

    /// Verifier-side evidence commitment.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }

    /// Aggregate proposal-source evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one exact proposal to the exact protected transition already qualified
/// by the repository layer, then independently verify its Git source claims.
#[allow(clippy::too_many_arguments)]
pub fn qualify_proposal_source<V: ProposalSourceVerifier>(
    proposal: &ChangeProposal,
    adoption: &RepositoryAdoption,
    repository_policy_state: &RepositoryPolicyState,
    project_policy: Digest,
    request: &RepositoryVerificationRequest,
    repository_verification: &QualifiedRepositoryVerification,
    observation: ProposalSourceObservation,
    verifier: &V,
) -> Result<QualifiedProposalSource, ProposalSourceError> {
    if proposal.project() != request.project() || proposal.project() != adoption.project() {
        return Err(ProposalSourceError::ProjectMismatch);
    }
    if proposal.target_ref() != request.reference() {
        return Err(ProposalSourceError::TargetRefMismatch);
    }
    if proposal.repository_policy_state() != request.repository_policy_state() {
        return Err(ProposalSourceError::RepositoryPolicyMismatch);
    }

    let commitment_algorithm = request.repository_policy_state().algorithm();
    let expected_request = RepositoryVerificationRequest::new(
        adoption,
        proposal.base_revision().clone(),
        proposal.proposed_revision().clone(),
        proposal.authority_epoch().clone(),
        project_policy,
        repository_policy_state,
        commitment_algorithm,
    )?;
    if expected_request.canonical_bytes()? != request.canonical_bytes()? {
        return Err(ProposalSourceError::RepositoryRequestMismatch);
    }

    let structural = repository_verification.structural();
    let expected_request_digest = request.digest(structural.request_digest().algorithm())?;
    if structural.request_digest() != &expected_request_digest {
        return Err(ProposalSourceError::RepositoryVerificationMismatch);
    }
    if structural.observed_tip() != proposal.proposed_revision() {
        return Err(ProposalSourceError::RepositoryTipMismatch);
    }
    if structural.repository_policy_state() != proposal.repository_policy_state() {
        return Err(ProposalSourceError::RepositoryPolicyMismatch);
    }

    if observation.adapter() != &verifier.identity() {
        return Err(ProposalSourceError::VerifierIdentityMismatch);
    }
    let expected_proposal = proposal.proposal_id(observation.proposal().commitment().algorithm())?;
    if observation.proposal() != &expected_proposal {
        return Err(ProposalSourceError::ProposalMismatch);
    }
    let observation_request = request.digest(observation.request_digest().algorithm())?;
    if observation.request_digest() != &observation_request {
        return Err(ProposalSourceError::ObservationRequestMismatch);
    }
    if observation.observed_base() != proposal.base_revision() {
        return Err(ProposalSourceError::ObservedBaseMismatch);
    }
    if observation.observed_proposed() != proposal.proposed_revision() {
        return Err(ProposalSourceError::ObservedProposedMismatch);
    }
    if observation.observed_tree() != proposal.resulting_tree() {
        return Err(ProposalSourceError::ObservedTreeMismatch);
    }

    let verifier_evidence = verifier.verify_source(proposal, request, &observation)?;
    let algorithm = verifier_evidence.algorithm();
    let repository_evidence = repository_verification_commitment(repository_verification, algorithm)?;
    let observation_digest = observation.digest(algorithm)?;
    let proposal_id = proposal.proposal_id(algorithm)?;
    let request_digest = request.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(QUALIFIED_SOURCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, proposal_id.commitment())?;
    push_digest(&mut out, &request_digest)?;
    push_digest(&mut out, &repository_evidence)?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, &verifier_evidence)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(QualifiedProposalSource {
        project: proposal.project().clone(),
        proposal: proposal_id,
        request_digest,
        repository_evidence,
        source_observation: observation,
        verifier_evidence,
        evidence_commitment,
    })
}

fn repository_verification_commitment(
    qualified: &QualifiedRepositoryVerification,
    algorithm: DigestAlgorithm,
) -> Result<Digest, ProposalSourceError> {
    let structural = qualified.structural();
    let mut out = Vec::new();
    out.extend_from_slice(REPOSITORY_EVIDENCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, structural.request_digest())?;
    push_bytes(&mut out, "repository_adapter_name", structural.adapter().name().as_bytes())?;
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
        ProposalSourceError::CanonicalFieldTooLarge {
            field: "repository_capabilities",
            len: structural.capabilities().len(),
            max: u16::MAX as usize,
        }
    })?;
    out.extend_from_slice(&count.to_be_bytes());
    for capability in structural.capabilities() {
        out.extend_from_slice(&verification_capability_code(*capability).to_be_bytes());
    }
    Ok(Digest::of_bytes(algorithm, &out))
}

/// Proposal-source binding failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ProposalSourceError {
    /// Proposal/request/adoption project mismatch.
    #[error("proposal source project mismatch")]
    ProjectMismatch,
    /// Proposal targets another protected ref.
    #[error("proposal target ref does not match repository verification request")]
    TargetRefMismatch,
    /// Proposal/request repository-policy state mismatch.
    #[error("proposal repository-policy state does not match repository request")]
    RepositoryPolicyMismatch,
    /// Reconstructed exact protected-transition request differs from the request that was verified.
    #[error("proposal does not reconstruct the exact repository verification request")]
    RepositoryRequestMismatch,
    /// Positive repository evidence belongs to another request.
    #[error("qualified repository verification belongs to another request")]
    RepositoryVerificationMismatch,
    /// Positive repository evidence observed another proposed tip.
    #[error("qualified repository verification observed another tip")]
    RepositoryTipMismatch,
    /// Git-source verifier implementation differs from observation metadata.
    #[error("proposal source verifier identity mismatch")]
    VerifierIdentityMismatch,
    /// Raw source observation names another proposal.
    #[error("proposal source observation names another proposal")]
    ProposalMismatch,
    /// Raw source observation names another protected request.
    #[error("proposal source observation names another repository request")]
    ObservationRequestMismatch,
    /// Raw source observation saw another base revision.
    #[error("proposal source observation base mismatch")]
    ObservedBaseMismatch,
    /// Raw source observation saw another proposed revision.
    #[error("proposal source observation proposed revision mismatch")]
    ObservedProposedMismatch,
    /// Raw source observation saw another resulting tree.
    #[error("proposal source observation tree mismatch")]
    ObservedTreeMismatch,
    /// Verifier identity field is invalid.
    #[error("invalid source verifier field {field}: length {len}")]
    InvalidVerifierField {
        /// Field name.
        field: &'static str,
        /// Observed byte length.
        len: usize,
    },
    /// Canonical field exceeded protocol encoding bounds.
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
    /// Repository request/canonicalization failed.
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    /// Concrete Git-source verifier rejected the observation.
    #[error(transparent)]
    Verifier(#[from] SourceVerifierError),
}

fn validate_adapter_field(field: &'static str, value: &str) -> Result<(), ProposalSourceError> {
    let len = value.as_bytes().len();
    if len == 0 || len > MAX_ADAPTER_FIELD_LEN {
        return Err(ProposalSourceError::InvalidVerifierField { field, len });
    }
    Ok(())
}

fn verification_capability_code(capability: VerificationCapability) -> u16 {
    match capability {
        VerificationCapability::RefTipBinding => 1,
        VerificationCapability::FullHistory => 2,
        VerificationCapability::ProtectedRewriteDetection => 3,
        VerificationCapability::PolicyLineageMonotonic => 4,
        VerificationCapability::AuthorizationAttestations => 5,
        VerificationCapability::OfflineEvidence => 6,
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ProposalSourceError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_git_object(out: &mut Vec<u8>, object: &GitObjectId) -> Result<(), ProposalSourceError> {
    out.push(match object.algorithm() {
        mycelix_forge_repository::GitObjectAlgorithm::Sha1 => 1,
        mycelix_forge_repository::GitObjectAlgorithm::Sha256 => 2,
    });
    push_bytes(out, "git_object", object.as_bytes())
}

fn push_optional_digest(
    out: &mut Vec<u8>,
    digest: Option<&Digest>,
) -> Result<(), ProposalSourceError> {
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

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ProposalSourceError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProposalSourceError> {
    let len = u32::try_from(bytes.len()).map_err(|_| ProposalSourceError::CanonicalFieldTooLarge {
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
    use mycelix_forge_authority::PrincipalId;
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_repository::{
        qualify_evidence_backed_observation, AdapterIdentity, AdapterObservation, AdapterOutcome,
        EvidenceBackedAdapterObservation, GitObjectAlgorithm, RepositoryRef, RepositoryTip,
        VerificationProfile,
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

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    struct Fixture {
        proposal: ChangeProposal,
        adoption: RepositoryAdoption,
        policy_state: RepositoryPolicyState,
        project_policy: Digest,
        request: RepositoryVerificationRequest,
        qualified_repository: QualifiedRepositoryVerification,
    }

    fn fixture() -> Fixture {
        let project = project();
        let authority_epoch = digest(0x30);
        let project_policy = digest(0x31);
        let external_policy = digest(0x32);
        let base = git(0x40);
        let proposed = git(0x41);
        let tree = git(0x42);
        let reference = RepositoryRef::new("refs/heads/main").unwrap();
        let adoption = RepositoryAdoption::new(
            project.clone(),
            RepositoryTip::new(reference.clone(), base.clone()),
            authority_epoch.clone(),
            project_policy.clone(),
            external_policy.clone(),
            100,
        );
        let policy_state =
            RepositoryPolicyState::new(project.clone(), 0, None, external_policy).unwrap();
        let request = RepositoryVerificationRequest::new(
            &adoption,
            base.clone(),
            proposed.clone(),
            authority_epoch.clone(),
            project_policy.clone(),
            &policy_state,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let proposal = ChangeProposal::new(
            project,
            PrincipalId::new(digest(0x20)),
            authority_epoch,
            request.repository_policy_state().clone(),
            reference,
            base,
            proposed,
            tree,
            digest(0x50),
            vec![],
        )
        .unwrap();

        let observation = AdapterObservation::new(
            AdapterIdentity::new("test-repository", "v1").unwrap(),
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            request.to().clone(),
            request.repository_policy_state().clone(),
            None,
            None,
            [VerificationCapability::RefTipBinding],
            AdapterOutcome::Verified,
        )
        .unwrap();
        let backed = EvidenceBackedAdapterObservation::new(observation, None).unwrap();
        let profile = VerificationProfile::new([VerificationCapability::RefTipBinding]).unwrap();
        let qualified_repository =
            qualify_evidence_backed_observation(&profile, &request, backed).unwrap();

        Fixture {
            proposal,
            adoption,
            policy_state,
            project_policy,
            request,
            qualified_repository,
        }
    }

    struct ExactSourceVerifier {
        identity: SourceVerifierIdentity,
        expected_source: Digest,
        evidence: Digest,
    }

    impl ProposalSourceVerifier for ExactSourceVerifier {
        fn identity(&self) -> SourceVerifierIdentity {
            self.identity.clone()
        }

        fn verify_source(
            &self,
            _proposal: &ChangeProposal,
            _request: &RepositoryVerificationRequest,
            observation: &ProposalSourceObservation,
        ) -> Result<Digest, SourceVerifierError> {
            if observation.source_state() != &self.expected_source {
                return Err(SourceVerifierError::Rejected);
            }
            Ok(self.evidence.clone())
        }
    }

    fn source_observation(f: &Fixture) -> (ProposalSourceObservation, ExactSourceVerifier) {
        let identity = SourceVerifierIdentity::new("test-git-source", "v1").unwrap();
        let source_state = digest(0x90);
        let observation = ProposalSourceObservation::new(
            identity.clone(),
            f.proposal.proposal_id(DigestAlgorithm::Sha256).unwrap(),
            f.request.digest(DigestAlgorithm::Sha256).unwrap(),
            source_state.clone(),
            f.proposal.base_revision().clone(),
            f.proposal.proposed_revision().clone(),
            f.proposal.resulting_tree().clone(),
            digest(0x91),
            digest(0x92),
        );
        let verifier = ExactSourceVerifier {
            identity,
            expected_source: source_state,
            evidence: digest(0x93),
        };
        (observation, verifier)
    }

    #[test]
    fn exact_proposal_source_qualifies() {
        let f = fixture();
        let (observation, verifier) = source_observation(&f);
        let qualified = qualify_proposal_source(
            &f.proposal,
            &f.adoption,
            &f.policy_state,
            f.project_policy.clone(),
            &f.request,
            &f.qualified_repository,
            observation,
            &verifier,
        )
        .unwrap();
        assert_eq!(qualified.project(), f.proposal.project());
        assert_eq!(qualified.source_observation().observed_tree(), f.proposal.resulting_tree());
    }

    #[test]
    fn tree_substitution_fails_before_verifier() {
        let f = fixture();
        let (mut observation, verifier) = source_observation(&f);
        observation.observed_tree = git(0x49);
        assert_eq!(
            qualify_proposal_source(
                &f.proposal,
                &f.adoption,
                &f.policy_state,
                f.project_policy.clone(),
                &f.request,
                &f.qualified_repository,
                observation,
                &verifier,
            )
            .unwrap_err(),
            ProposalSourceError::ObservedTreeMismatch
        );
    }

    #[test]
    fn reconstructed_repository_request_must_match_exactly() {
        let f = fixture();
        let other_request = RepositoryVerificationRequest::new(
            &f.adoption,
            f.proposal.base_revision().clone(),
            f.proposal.proposed_revision().clone(),
            digest(0x39),
            f.project_policy.clone(),
            &f.policy_state,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let observation = AdapterObservation::new(
            AdapterIdentity::new("test-repository", "v1").unwrap(),
            other_request.digest(DigestAlgorithm::Sha256).unwrap(),
            other_request.to().clone(),
            other_request.repository_policy_state().clone(),
            None,
            None,
            [VerificationCapability::RefTipBinding],
            AdapterOutcome::Verified,
        )
        .unwrap();
        let backed = EvidenceBackedAdapterObservation::new(observation, None).unwrap();
        let profile = VerificationProfile::new([VerificationCapability::RefTipBinding]).unwrap();
        let qualified = qualify_evidence_backed_observation(&profile, &other_request, backed).unwrap();
        let (source, verifier) = source_observation(&f);
        assert_eq!(
            qualify_proposal_source(
                &f.proposal,
                &f.adoption,
                &f.policy_state,
                f.project_policy.clone(),
                &other_request,
                &qualified,
                source,
                &verifier,
            )
            .unwrap_err(),
            ProposalSourceError::RepositoryRequestMismatch
        );
    }

    #[test]
    fn wrong_source_state_is_rejected_by_verifier() {
        let f = fixture();
        let (observation, mut verifier) = source_observation(&f);
        verifier.expected_source = digest(0x99);
        assert_eq!(
            qualify_proposal_source(
                &f.proposal,
                &f.adoption,
                &f.policy_state,
                f.project_policy.clone(),
                &f.request,
                &f.qualified_repository,
                observation,
                &verifier,
            )
            .unwrap_err(),
            ProposalSourceError::Verifier(SourceVerifierError::Rejected)
        );
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable exact-subject change proposals for Mycelix Forge.
//!
//! A proposal is the immutable review subject for one source transition. It
//! binds the exact project, claimed proposer, typed authority/project-policy/
//! repository-policy context, target ref, base/proposed commits, claimed
//! resulting tree, immutable change intent, and typed proposal dependencies.
//! Mutable discussion/UI metadata is deliberately excluded.
//!
//! This crate does **not** prove that the proposed commit has the claimed tree,
//! authenticate the proposer, establish repository-policy compliance, or grant
//! review/merge authority. Those are separate Forge evidence layers.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, PrincipalId};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_project_policy::{ProjectPolicyError, ProjectPolicyStateV1};
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    RepositoryVerificationError,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const CHANGE_PROPOSAL_DOMAIN_V1: &[u8] = b"mycelix-forge/change-proposal/v1\0";
const MAX_DEPENDENCIES_V1: usize = 256;

/// Stable identifier for one exact immutable change proposal.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ChangeProposalId(Digest);

impl ChangeProposalId {
    /// Construct from an already-derived proposal commitment.
    pub fn new(commitment: Digest) -> Self {
        Self(commitment)
    }

    /// Algorithm-qualified proposal commitment.
    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

/// Immutable review subject for one proposed source transition.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ChangeProposal {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposer: PrincipalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    target_ref: RepositoryRef,
    base_revision: GitObjectId,
    proposed_revision: GitObjectId,
    resulting_tree: GitObjectId,
    change_intent: Digest,
    dependencies: Vec<ChangeProposalId>,
}

impl ChangeProposal {
    /// Construct a v1 proposal from live typed authority, project-policy, and
    /// repository-policy context.
    ///
    /// Construction proves only that all three supplied context objects belong
    /// to the same project and that their exact commitments are embedded in
    /// the proposal. It does not authenticate or authorize `proposer`.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        proposer: PrincipalId,
        authority_epoch: &AuthorityEpoch,
        project_policy: &ProjectPolicyStateV1,
        repository_policy_state: &RepositoryPolicyState,
        target_ref: RepositoryRef,
        base_revision: GitObjectId,
        proposed_revision: GitObjectId,
        resulting_tree: GitObjectId,
        change_intent: Digest,
        dependencies: Vec<ChangeProposalId>,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ProposalError> {
        let project = authority_epoch.project().clone();
        if project_policy.project() != &project {
            return Err(ProposalError::ProjectPolicyProjectMismatch);
        }
        if repository_policy_state.project() != &project {
            return Err(ProposalError::RepositoryPolicyProjectMismatch);
        }

        let authority_epoch = authority_epoch.digest(commitment_algorithm)?;
        let project_policy = project_policy.digest(commitment_algorithm)?;
        let repository_policy_state = repository_policy_state.digest(commitment_algorithm)?;
        let dependencies = normalize_dependencies(dependencies)?;

        Self::from_validated_parts(
            project,
            proposer,
            authority_epoch,
            project_policy,
            repository_policy_state,
            target_ref,
            base_revision,
            proposed_revision,
            resulting_tree,
            change_intent,
            dependencies,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_validated_parts(
        project: ProjectIdentity,
        proposer: PrincipalId,
        authority_epoch: Digest,
        project_policy: Digest,
        repository_policy_state: Digest,
        target_ref: RepositoryRef,
        base_revision: GitObjectId,
        proposed_revision: GitObjectId,
        resulting_tree: GitObjectId,
        change_intent: Digest,
        dependencies: Vec<ChangeProposalId>,
    ) -> Result<Self, ProposalError> {
        if base_revision == proposed_revision {
            return Err(ProposalError::NoOpTransition);
        }

        let object_algorithm = base_revision.algorithm();
        for (field, object) in [
            ("proposed_revision", &proposed_revision),
            ("resulting_tree", &resulting_tree),
        ] {
            if object.algorithm() != object_algorithm {
                return Err(ProposalError::MixedGitObjectAlgorithms {
                    field,
                    expected: object_algorithm,
                    actual: object.algorithm(),
                });
            }
        }

        if dependencies.len() > MAX_DEPENDENCIES_V1 {
            return Err(ProposalError::TooManyDependencies {
                actual: dependencies.len(),
                max: MAX_DEPENDENCIES_V1,
            });
        }
        if dependencies.windows(2).any(|pair| pair[0] >= pair[1]) {
            return Err(ProposalError::NonCanonicalDependencies);
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            proposer,
            authority_epoch,
            project_policy,
            repository_policy_state,
            target_ref,
            base_revision,
            proposed_revision,
            resulting_tree,
            change_intent,
            dependencies,
        })
    }

    /// Forge protocol version.
    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }

    /// Project this proposal belongs to.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Stable Forge principal claiming authorship of this proposal.
    pub fn proposer(&self) -> &PrincipalId {
        &self.proposer
    }

    /// Exact authority epoch in which this proposal is interpreted.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact typed project-policy-state context relevant to review/acceptance.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state relevant to review/acceptance.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Protected/target ref for the proposed transition.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Exact commit/ref tip from which the proposal starts.
    pub fn base_revision(&self) -> &GitObjectId {
        &self.base_revision
    }

    /// Exact proposed commit/ref tip.
    pub fn proposed_revision(&self) -> &GitObjectId {
        &self.proposed_revision
    }

    /// Claimed exact tree resulting from `proposed_revision`.
    ///
    /// The proposal binds this claim but does not itself prove Git object type
    /// or commit→tree correspondence; repository evidence must establish that.
    pub fn resulting_tree(&self) -> &GitObjectId {
        &self.resulting_tree
    }

    /// Immutable requirements/motivation/specification commitment.
    pub fn change_intent(&self) -> &Digest {
        &self.change_intent
    }

    /// Sorted unique typed proposal dependencies.
    pub fn dependencies(&self) -> &[ChangeProposalId] {
        &self.dependencies
    }

    /// Repository Git object format used by this proposal.
    pub const fn git_object_algorithm(&self) -> GitObjectAlgorithm {
        self.base_revision.algorithm()
    }

    /// Canonical v1 bytes identifying the exact review subject.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProposalError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(CHANGE_PROPOSAL_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposer.commitment())?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.project_policy)?;
        push_digest(&mut out, &self.repository_policy_state)?;
        push_ref(&mut out, &self.target_ref)?;
        push_git_object(&mut out, &self.base_revision)?;
        push_git_object(&mut out, &self.proposed_revision)?;
        push_git_object(&mut out, &self.resulting_tree)?;
        push_digest(&mut out, &self.change_intent)?;
        let count = u16::try_from(self.dependencies.len()).map_err(|_| {
            ProposalError::CanonicalFieldTooLarge {
                field: "dependencies",
                len: self.dependencies.len(),
                max: u16::MAX as usize,
            }
        })?;
        out.extend_from_slice(&count.to_be_bytes());
        for dependency in &self.dependencies {
            push_digest(&mut out, dependency.commitment())?;
        }
        Ok(out)
    }

    /// Derive the stable proposal identifier with an explicit Forge digest suite.
    pub fn proposal_id(&self, algorithm: DigestAlgorithm) -> Result<ChangeProposalId, ProposalError> {
        Ok(ChangeProposalId::new(Digest::of_bytes(
            algorithm,
            &self.canonical_bytes()?,
        )))
    }
}

impl<'de> Deserialize<'de> for ChangeProposal {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireProposal {
            version: ProtocolVersion,
            project: ProjectIdentity,
            proposer: PrincipalId,
            authority_epoch: Digest,
            project_policy: Digest,
            repository_policy_state: Digest,
            target_ref: RepositoryRef,
            base_revision: GitObjectId,
            proposed_revision: GitObjectId,
            resulting_tree: GitObjectId,
            change_intent: Digest,
            dependencies: Vec<ChangeProposalId>,
        }

        let wire = WireProposal::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_validated_parts(
            wire.project,
            wire.proposer,
            wire.authority_epoch,
            wire.project_policy,
            wire.repository_policy_state,
            wire.target_ref,
            wire.base_revision,
            wire.proposed_revision,
            wire.resulting_tree,
            wire.change_intent,
            wire.dependencies,
        )
        .map_err(D::Error::custom)
    }
}

/// Change-proposal validation/canonicalization failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ProposalError {
    /// Unsupported proposal protocol version.
    #[error("unsupported Forge change-proposal protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Typed project-policy state belongs to a different project than the authority epoch.
    #[error("project-policy state project does not match authority-epoch project")]
    ProjectPolicyProjectMismatch,
    /// Typed repository-policy state belongs to a different project than the authority epoch.
    #[error("repository-policy state project does not match authority-epoch project")]
    RepositoryPolicyProjectMismatch,
    /// A proposal must actually change the target revision.
    #[error("change proposal base and proposed revisions are identical")]
    NoOpTransition,
    /// Git objects in one proposal must use one object-format algorithm.
    #[error("{field} uses {actual:?}, expected repository object format {expected:?}")]
    MixedGitObjectAlgorithms {
        /// Field whose algorithm differed.
        field: &'static str,
        /// Expected format from the base revision.
        expected: GitObjectAlgorithm,
        /// Actual format.
        actual: GitObjectAlgorithm,
    },
    /// Proposal dependencies must be strictly ordered and unique on the wire.
    #[error("change proposal dependencies are duplicated or non-canonical")]
    NonCanonicalDependencies,
    /// Proposal dependency bound exceeded.
    #[error("change proposal has too many dependencies: {actual} > {max}")]
    TooManyDependencies {
        /// Observed number.
        actual: usize,
        /// Protocol-v1 limit.
        max: usize,
    },
    /// Canonical field exceeded its encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum length.
        max: usize,
    },
    /// Authority-state validation/canonicalization failed.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
    /// Project-policy-state validation/canonicalization failed.
    #[error(transparent)]
    ProjectPolicy(#[from] ProjectPolicyError),
    /// Repository-policy-state validation/canonicalization failed.
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
}

fn normalize_dependencies(
    mut dependencies: Vec<ChangeProposalId>,
) -> Result<Vec<ChangeProposalId>, ProposalError> {
    if dependencies.len() > MAX_DEPENDENCIES_V1 {
        return Err(ProposalError::TooManyDependencies {
            actual: dependencies.len(),
            max: MAX_DEPENDENCIES_V1,
        });
    }
    dependencies.sort();
    if dependencies.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(ProposalError::NonCanonicalDependencies);
    }
    Ok(dependencies)
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ProposalError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProposalError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_project_identity(out: &mut Vec<u8>, project: &ProjectIdentity) -> Result<(), ProposalError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_ref(out: &mut Vec<u8>, reference: &RepositoryRef) -> Result<(), ProposalError> {
    push_bytes(out, "target_ref", reference.as_str().as_bytes())
}

fn push_git_object(out: &mut Vec<u8>, object: &GitObjectId) -> Result<(), ProposalError> {
    out.push(object.algorithm().code());
    push_bytes(out, "git_object", object.as_bytes())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ProposalError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(out: &mut Vec<u8>, field: &'static str, bytes: &[u8]) -> Result<(), ProposalError> {
    let len = u32::try_from(bytes.len()).map_err(|_| ProposalError::CanonicalFieldTooLarge {
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
    use mycelix_forge_authority::{AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant};
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, TrustedProviderVerifierV1,
    };

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project(nonce: u8) -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([nonce; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn authority(project: ProjectIdentity, manager: PrincipalId, valid_from: u64) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project,
            sequence: 0,
            previous: None,
            valid_from_unix_ms: valid_from,
            valid_until_unix_ms: None,
            grants: vec![PrincipalGrant::new(manager, [Capability::ManageAuthority]).unwrap()],
            thresholds: vec![CapabilityRule::new(Capability::ManageAuthority, 1).unwrap()],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn repository_policy(project: ProjectIdentity) -> RepositoryPolicyState {
        RepositoryPolicyState::new(project, 0, None, digest(0x31)).unwrap()
    }

    fn project_policy(project: ProjectIdentity, verifier_marker: u8) -> ProjectPolicyStateV1 {
        let trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![TrustedProviderVerifierV1::new(
                digest(0x90),
                digest(verifier_marker),
            )],
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

    fn dep(byte: u8) -> ChangeProposalId {
        ChangeProposalId::new(digest(byte))
    }

    fn proposal() -> ChangeProposal {
        let project = project(0x11);
        let authority = authority(project.clone(), principal(0x20), 1_000);
        let project_policy = project_policy(project.clone(), 0x91);
        let repository_policy = repository_policy(project);
        ChangeProposal::new(
            principal(0x21),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![dep(0x61), dep(0x60)],
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    #[test]
    fn dependencies_are_normalized_and_identity_is_stable() {
        let p = proposal();
        assert_eq!(p.dependencies(), &[dep(0x60), dep(0x61)]);
        let json = serde_json::to_vec(&p).unwrap();
        let decoded: ChangeProposal = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, p);
        assert_eq!(
            decoded.proposal_id(DigestAlgorithm::Sha256).unwrap(),
            p.proposal_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn serde_rejects_noncanonical_dependency_order() {
        let p = proposal();
        let mut value = serde_json::to_value(&p).unwrap();
        value["dependencies"].as_array_mut().unwrap().reverse();
        assert!(serde_json::from_value::<ChangeProposal>(value).is_err());
    }

    #[test]
    fn proposed_revision_and_resulting_tree_mutations_change_identity() {
        let project = project(0x11);
        let authority = authority(project.clone(), principal(0x20), 1_000);
        let project_policy = project_policy(project.clone(), 0x91);
        let repository_policy = repository_policy(project);
        let baseline = proposal();
        for (proposed_revision, resulting_tree) in [(git(0x43), git(0x42)), (git(0x41), git(0x44))] {
            let mutated = ChangeProposal::new(
                baseline.proposer().clone(),
                &authority,
                &project_policy,
                &repository_policy,
                baseline.target_ref().clone(),
                baseline.base_revision().clone(),
                proposed_revision,
                resulting_tree,
                baseline.change_intent().clone(),
                baseline.dependencies().to_vec(),
                DigestAlgorithm::Sha256,
            )
            .unwrap();
            assert_ne!(
                baseline.proposal_id(DigestAlgorithm::Sha256).unwrap(),
                mutated.proposal_id(DigestAlgorithm::Sha256).unwrap()
            );
        }
    }

    #[test]
    fn policy_authority_intent_ref_author_and_dependency_changes_change_identity() {
        let baseline = proposal();
        let project = project(0x11);
        let base_repository_policy = repository_policy(project.clone());
        let base_project_policy = project_policy(project.clone(), 0x91);
        let base_authority = authority(project.clone(), principal(0x20), 1_000);
        let changed_authority = authority(project.clone(), principal(0x20), 1_001);
        let changed_project_policy = project_policy(project.clone(), 0x92);
        let changed_repository_policy =
            RepositoryPolicyState::new(project.clone(), 0, None, digest(0x39)).unwrap();

        let cases = vec![
            ChangeProposal::new(
                principal(0x29), &base_authority, &base_project_policy, &base_repository_policy,
                baseline.target_ref().clone(), baseline.base_revision().clone(), baseline.proposed_revision().clone(), baseline.resulting_tree().clone(), baseline.change_intent().clone(), baseline.dependencies().to_vec(), DigestAlgorithm::Sha256,
            ).unwrap(),
            ChangeProposal::new(
                baseline.proposer().clone(), &changed_authority, &base_project_policy, &base_repository_policy,
                baseline.target_ref().clone(), baseline.base_revision().clone(), baseline.proposed_revision().clone(), baseline.resulting_tree().clone(), baseline.change_intent().clone(), baseline.dependencies().to_vec(), DigestAlgorithm::Sha256,
            ).unwrap(),
            ChangeProposal::new(
                baseline.proposer().clone(), &base_authority, &changed_project_policy, &base_repository_policy,
                baseline.target_ref().clone(), baseline.base_revision().clone(), baseline.proposed_revision().clone(), baseline.resulting_tree().clone(), baseline.change_intent().clone(), baseline.dependencies().to_vec(), DigestAlgorithm::Sha256,
            ).unwrap(),
            ChangeProposal::new(
                baseline.proposer().clone(), &base_authority, &base_project_policy, &changed_repository_policy,
                baseline.target_ref().clone(), baseline.base_revision().clone(), baseline.proposed_revision().clone(), baseline.resulting_tree().clone(), baseline.change_intent().clone(), baseline.dependencies().to_vec(), DigestAlgorithm::Sha256,
            ).unwrap(),
            ChangeProposal::new(
                baseline.proposer().clone(), &base_authority, &base_project_policy, &base_repository_policy,
                RepositoryRef::new("refs/heads/release").unwrap(), baseline.base_revision().clone(), baseline.proposed_revision().clone(), baseline.resulting_tree().clone(), baseline.change_intent().clone(), baseline.dependencies().to_vec(), DigestAlgorithm::Sha256,
            ).unwrap(),
            ChangeProposal::new(
                baseline.proposer().clone(), &base_authority, &base_project_policy, &base_repository_policy,
                baseline.target_ref().clone(), baseline.base_revision().clone(), baseline.proposed_revision().clone(), baseline.resulting_tree().clone(), digest(0x5a), baseline.dependencies().to_vec(), DigestAlgorithm::Sha256,
            ).unwrap(),
            ChangeProposal::new(
                baseline.proposer().clone(), &base_authority, &base_project_policy, &base_repository_policy,
                baseline.target_ref().clone(), baseline.base_revision().clone(), baseline.proposed_revision().clone(), baseline.resulting_tree().clone(), baseline.change_intent().clone(), vec![dep(0x60), dep(0x62)], DigestAlgorithm::Sha256,
            ).unwrap(),
        ];

        for mutated in cases {
            assert_ne!(
                baseline.proposal_id(DigestAlgorithm::Sha256).unwrap(),
                mutated.proposal_id(DigestAlgorithm::Sha256).unwrap()
            );
        }
    }

    #[test]
    fn no_op_and_mixed_object_formats_fail_closed() {
        let project = project(0x11);
        let authority = authority(project.clone(), principal(0x20), 1_000);
        let project_policy = project_policy(project.clone(), 0x91);
        let repository_policy = repository_policy(project);
        let base = git(0x40);
        let common = |proposed_revision: GitObjectId, resulting_tree: GitObjectId| {
            ChangeProposal::new(
                principal(0x21),
                &authority,
                &project_policy,
                &repository_policy,
                RepositoryRef::new("refs/heads/main").unwrap(),
                base.clone(),
                proposed_revision,
                resulting_tree,
                digest(0x50),
                vec![],
                DigestAlgorithm::Sha256,
            )
        };
        assert_eq!(common(base.clone(), git(0x42)).unwrap_err(), ProposalError::NoOpTransition);
        let sha256 = GitObjectId::new(GitObjectAlgorithm::Sha256, vec![0x41; 32]).unwrap();
        assert!(matches!(
            common(sha256, git(0x42)).unwrap_err(),
            ProposalError::MixedGitObjectAlgorithms { .. }
        ));
    }

    #[test]
    fn typed_context_rejects_cross_project_project_and_repository_policy() {
        let project_a = project(0x11);
        let project_b = project(0x13);
        let authority = authority(project_a.clone(), principal(0x20), 1_000);
        let project_policy_a = project_policy(project_a.clone(), 0x91);
        let project_policy_b = project_policy(project_b.clone(), 0x91);
        let repository_policy_a = repository_policy(project_a);
        let repository_policy_b = repository_policy(project_b);

        let project_policy_error = ChangeProposal::new(
            principal(0x21),
            &authority,
            &project_policy_b,
            &repository_policy_a,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap_err();
        assert_eq!(project_policy_error, ProposalError::ProjectPolicyProjectMismatch);

        let repository_policy_error = ChangeProposal::new(
            principal(0x21),
            &authority,
            &project_policy_a,
            &repository_policy_b,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap_err();
        assert_eq!(repository_policy_error, ProposalError::RepositoryPolicyProjectMismatch);
    }

    #[test]
    fn duplicate_dependencies_fail_instead_of_silent_deduplication() {
        let project = project(0x11);
        let authority = authority(project.clone(), principal(0x20), 1_000);
        let project_policy = project_policy(project.clone(), 0x91);
        let repository_policy = repository_policy(project);
        let dependency = dep(0x60);
        let error = ChangeProposal::new(
            principal(0x21),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![dependency.clone(), dependency],
            DigestAlgorithm::Sha256,
        )
        .unwrap_err();
        assert_eq!(error, ProposalError::NonCanonicalDependencies);
    }

    #[test]
    fn naming_a_proposer_does_not_authenticate_or_authorize_them() {
        let p = proposal();
        let project = p.project().clone();
        let manager = principal(0x20);
        let authority = authority(project, manager, 1_000);
        assert!(!authority.is_principal_eligible(
            p.proposer(),
            mycelix_forge_authority::Capability::ReviewSource,
            1_000
        ));
    }
}

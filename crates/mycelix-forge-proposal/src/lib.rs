// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable exact-subject change proposals for Mycelix Forge.
//!
//! A proposal is the immutable review subject for one source transition. It
//! binds the exact project, proposer, authority/policy context, target ref,
//! base/proposed commits, claimed resulting tree, immutable change intent and
//! dependencies. Mutable discussion/UI metadata is deliberately excluded.
//!
//! This crate does **not** prove that the proposed commit has the claimed tree,
//! authenticate the proposer, establish repository-policy compliance, or grant
//! merge authority. Those are separate Forge evidence layers.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::PrincipalId;
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::collections::BTreeSet;
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
    repository_policy_state: Digest,
    target_ref: RepositoryRef,
    base_revision: GitObjectId,
    proposed_revision: GitObjectId,
    resulting_tree: GitObjectId,
    change_intent: Digest,
    dependencies: Vec<Digest>,
}

impl ChangeProposal {
    /// Construct and normalize a v1 change proposal.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        project: ProjectIdentity,
        proposer: PrincipalId,
        authority_epoch: Digest,
        repository_policy_state: Digest,
        target_ref: RepositoryRef,
        base_revision: GitObjectId,
        proposed_revision: GitObjectId,
        resulting_tree: GitObjectId,
        change_intent: Digest,
        dependencies: Vec<Digest>,
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

        let unique = dependencies.iter().cloned().collect::<BTreeSet<_>>();
        if unique.len() != dependencies.len() {
            return Err(ProposalError::DuplicateDependency);
        }
        let dependencies = unique.into_iter().collect();

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            proposer,
            authority_epoch,
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

    /// Exact authority epoch in which this proposal was created.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
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

    /// Sorted unique proposal dependencies.
    pub fn dependencies(&self) -> &[Digest] {
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
            push_digest(&mut out, dependency)?;
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
            repository_policy_state: Digest,
            target_ref: RepositoryRef,
            base_revision: GitObjectId,
            proposed_revision: GitObjectId,
            resulting_tree: GitObjectId,
            change_intent: Digest,
            dependencies: Vec<Digest>,
        }

        let wire = WireProposal::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::new(
            wire.project,
            wire.proposer,
            wire.authority_epoch,
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
    /// Proposal dependency list contains the same proposal more than once.
    #[error("change proposal contains a duplicate dependency")]
    DuplicateDependency,
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

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn proposal() -> ChangeProposal {
        ChangeProposal::new(
            project(),
            PrincipalId::new(digest(0x20)),
            digest(0x30),
            digest(0x31),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![digest(0x61), digest(0x60)],
        )
        .unwrap()
    }

    #[test]
    fn dependencies_are_normalized_and_identity_is_stable() {
        let p = proposal();
        assert_eq!(p.dependencies(), &[digest(0x60), digest(0x61)]);
        let json = serde_json::to_vec(&p).unwrap();
        let decoded: ChangeProposal = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, p);
        assert_eq!(
            decoded.proposal_id(DigestAlgorithm::Sha256).unwrap(),
            p.proposal_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn proposed_revision_mutation_changes_identity() {
        let p = proposal();
        let mutated = ChangeProposal::new(
            p.project().clone(),
            p.proposer().clone(),
            p.authority_epoch().clone(),
            p.repository_policy_state().clone(),
            p.target_ref().clone(),
            p.base_revision().clone(),
            git(0x43),
            p.resulting_tree().clone(),
            p.change_intent().clone(),
            p.dependencies().to_vec(),
        )
        .unwrap();
        assert_ne!(
            p.proposal_id(DigestAlgorithm::Sha256).unwrap(),
            mutated.proposal_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn resulting_tree_mutation_changes_identity() {
        let p = proposal();
        let mutated = ChangeProposal::new(
            p.project().clone(),
            p.proposer().clone(),
            p.authority_epoch().clone(),
            p.repository_policy_state().clone(),
            p.target_ref().clone(),
            p.base_revision().clone(),
            p.proposed_revision().clone(),
            git(0x44),
            p.change_intent().clone(),
            p.dependencies().to_vec(),
        )
        .unwrap();
        assert_ne!(
            p.proposal_id(DigestAlgorithm::Sha256).unwrap(),
            mutated.proposal_id(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn intent_and_policy_mutations_change_identity() {
        let p = proposal();
        for (intent, policy) in [
            (digest(0x55), p.repository_policy_state().clone()),
            (p.change_intent().clone(), digest(0x56)),
        ] {
            let mutated = ChangeProposal::new(
                p.project().clone(),
                p.proposer().clone(),
                p.authority_epoch().clone(),
                policy,
                p.target_ref().clone(),
                p.base_revision().clone(),
                p.proposed_revision().clone(),
                p.resulting_tree().clone(),
                intent,
                p.dependencies().to_vec(),
            )
            .unwrap();
            assert_ne!(
                p.proposal_id(DigestAlgorithm::Sha256).unwrap(),
                mutated.proposal_id(DigestAlgorithm::Sha256).unwrap()
            );
        }
    }

    #[test]
    fn duplicate_dependencies_fail_instead_of_silent_deduplication() {
        let dep = digest(0x60);
        let error = ChangeProposal::new(
            project(),
            PrincipalId::new(digest(0x20)),
            digest(0x30),
            digest(0x31),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![dep.clone(), dep],
        )
        .unwrap_err();
        assert_eq!(error, ProposalError::DuplicateDependency);
    }

    #[test]
    fn no_op_and_mixed_object_formats_fail() {
        let same = git(0x40);
        assert_eq!(
            ChangeProposal::new(
                project(),
                PrincipalId::new(digest(0x20)),
                digest(0x30),
                digest(0x31),
                RepositoryRef::new("refs/heads/main").unwrap(),
                same.clone(),
                same,
                git(0x42),
                digest(0x50),
                vec![],
            )
            .unwrap_err(),
            ProposalError::NoOpTransition
        );

        let sha256_tree = GitObjectId::new(GitObjectAlgorithm::Sha256, vec![0x42; 32]).unwrap();
        assert!(matches!(
            ChangeProposal::new(
                project(),
                PrincipalId::new(digest(0x20)),
                digest(0x30),
                digest(0x31),
                RepositoryRef::new("refs/heads/main").unwrap(),
                git(0x40),
                git(0x41),
                sha256_tree,
                digest(0x50),
                vec![],
            ),
            Err(ProposalError::MixedGitObjectAlgorithms {
                field: "resulting_tree",
                ..
            })
        ));
    }
}

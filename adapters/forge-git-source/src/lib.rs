// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Concrete Git-plumbing verifier for FORGE-008A proposal-source evidence.
//!
//! The adapter proves Git semantics for the exact content-addressed proposal
//! objects. It does not claim the repository/object database is self-contained;
//! portable closure remains a separate Forge bundle/execution theorem.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_proposal::ChangeProposal;
use mycelix_forge_proposal_source::{
    ProposalSourceObservation, ProposalSourceVerifier, SourceVerifierError, SourceVerifierIdentity,
};
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryVerificationRequest,
};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use thiserror::Error;

const SOURCE_STATE_DOMAIN_V1: &[u8] = b"mycelix-forge/git-relevant-source-state/v1\0";
const ANCESTRY_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/git-ancestry-evidence/v1\0";
const TREE_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/git-tree-evidence/v1\0";
const VERIFIER_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/git-source-verifier-evidence/v1\0";

/// Captured command result from a Git runner.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitCommandResult {
    /// Process exit code, or `-1` if the platform did not expose one.
    pub status: i32,
    /// Captured stdout bytes.
    pub stdout: Vec<u8>,
    /// Captured stderr bytes.
    pub stderr: Vec<u8>,
}

/// Minimal runner abstraction used to test the adapter without executing Git.
pub trait GitCommandRunner {
    /// Execute Git with exact arguments against `repository`.
    fn run(
        &self,
        repository: &Path,
        args: &[String],
    ) -> Result<GitCommandResult, GitSourceAdapterError>;
}

/// Process-backed Git runner with a fixed executable path and cleared ambient
/// Git configuration/network-assisted object fetching.
#[derive(Clone, Debug)]
pub struct ProcessGitRunner {
    executable: PathBuf,
}

impl ProcessGitRunner {
    /// Construct a process runner. The executable must be an absolute path so
    /// qualification never searches ambient `PATH`.
    pub fn new(executable: PathBuf) -> Result<Self, GitSourceAdapterError> {
        if !executable.is_absolute() {
            return Err(GitSourceAdapterError::GitExecutableNotAbsolute);
        }
        Ok(Self { executable })
    }

    /// Exact Git executable path.
    pub fn executable(&self) -> &Path {
        &self.executable
    }
}

impl GitCommandRunner for ProcessGitRunner {
    fn run(
        &self,
        repository: &Path,
        args: &[String],
    ) -> Result<GitCommandResult, GitSourceAdapterError> {
        let output = Command::new(&self.executable)
            .env_clear()
            .env("GIT_CONFIG_NOSYSTEM", "1")
            .env("GIT_CONFIG_GLOBAL", "/dev/null")
            .env("GIT_TERMINAL_PROMPT", "0")
            .env("GIT_NO_LAZY_FETCH", "1")
            .env("GIT_NO_REPLACE_OBJECTS", "1")
            .arg("-C")
            .arg(repository)
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .map_err(|error| GitSourceAdapterError::Process(error.to_string()))?;

        Ok(GitCommandResult {
            status: output.status.code().unwrap_or(-1),
            stdout: output.stdout,
            stderr: output.stderr,
        })
    }
}

/// Concrete FORGE-008A Git source verifier.
#[derive(Clone, Debug)]
pub struct GitProposalSourceAdapter<R> {
    runner: R,
    repository: PathBuf,
    expected_git_version: String,
    identity: SourceVerifierIdentity,
}

impl<R> GitProposalSourceAdapter<R>
where
    R: GitCommandRunner,
{
    /// Construct an adapter pinned to an exact Git version string.
    pub fn new(
        runner: R,
        repository: PathBuf,
        expected_git_version: impl Into<String>,
    ) -> Result<Self, GitSourceAdapterError> {
        let expected_git_version = expected_git_version.into();
        if expected_git_version.is_empty() || expected_git_version.len() > 128 {
            return Err(GitSourceAdapterError::InvalidGitVersion);
        }
        let identity = SourceVerifierIdentity::new("git-plumbing", expected_git_version.clone())
            .map_err(|_| GitSourceAdapterError::InvalidGitVersion)?;
        Ok(Self {
            runner,
            repository,
            expected_git_version,
            identity,
        })
    }

    /// Build a raw FORGE-008A observation by interrogating Git.
    pub fn observe(
        &self,
        proposal: &ChangeProposal,
        request: &RepositoryVerificationRequest,
    ) -> Result<ProposalSourceObservation, GitSourceAdapterError> {
        let inspected = self.inspect(proposal)?;
        let proposal_id = proposal
            .proposal_id(DigestAlgorithm::Sha256)
            .map_err(|error| GitSourceAdapterError::Protocol(error.to_string()))?;
        let request_digest = request
            .digest(DigestAlgorithm::Sha256)
            .map_err(|error| GitSourceAdapterError::Protocol(error.to_string()))?;
        Ok(ProposalSourceObservation::new(
            self.identity.clone(),
            proposal_id,
            request_digest,
            inspected.source_state,
            proposal.base_revision().clone(),
            proposal.proposed_revision().clone(),
            inspected.tree,
            inspected.ancestry_evidence,
            inspected.tree_evidence,
        ))
    }

    fn inspect(&self, proposal: &ChangeProposal) -> Result<InspectedSource, GitSourceAdapterError> {
        self.verify_git_version()?;
        self.verify_object_format(proposal.git_object_algorithm())?;

        let base_hex = object_hex(proposal.base_revision());
        let proposed_hex = object_hex(proposal.proposed_revision());
        let tree_hex = object_hex(proposal.resulting_tree());

        self.require_stdout(
            &["--no-replace-objects", "cat-file", "-t", &base_hex],
            "commit",
            GitSourceAdapterError::BaseNotCommit,
        )?;
        self.require_stdout(
            &["--no-replace-objects", "cat-file", "-t", &proposed_hex],
            "commit",
            GitSourceAdapterError::ProposedNotCommit,
        )?;
        self.require_stdout(
            &["--no-replace-objects", "cat-file", "-t", &tree_hex],
            "tree",
            GitSourceAdapterError::ResultNotTree,
        )?;

        let tree_expr = format!("{proposed_hex}^{{tree}}");
        let observed_tree = self.run_success(&[
            "--no-replace-objects".to_string(),
            "rev-parse".to_string(),
            tree_expr,
        ])?;
        if trim_ascii(&observed_tree.stdout) != tree_hex.as_bytes() {
            return Err(GitSourceAdapterError::TreeMismatch);
        }

        let ancestry = self.runner.run(
            &self.repository,
            &[
                "--no-replace-objects".to_string(),
                "merge-base".to_string(),
                "--is-ancestor".to_string(),
                base_hex.clone(),
                proposed_hex.clone(),
            ],
        )?;
        if ancestry.status != 0 {
            return Err(GitSourceAdapterError::BaseNotAncestor);
        }

        Ok(InspectedSource {
            tree: proposal.resulting_tree().clone(),
            source_state: relevant_source_state(
                proposal.base_revision(),
                proposal.proposed_revision(),
                proposal.resulting_tree(),
            ),
            ancestry_evidence: ancestry_evidence(
                proposal.base_revision(),
                proposal.proposed_revision(),
            ),
            tree_evidence: tree_evidence(proposal.proposed_revision(), proposal.resulting_tree()),
        })
    }

    fn verify_git_version(&self) -> Result<(), GitSourceAdapterError> {
        let output = self.run_success(&["--version".to_string()])?;
        let expected = format!("git version {}", self.expected_git_version);
        if trim_ascii(&output.stdout) != expected.as_bytes() {
            return Err(GitSourceAdapterError::GitVersionMismatch);
        }
        Ok(())
    }

    fn verify_object_format(
        &self,
        expected: GitObjectAlgorithm,
    ) -> Result<(), GitSourceAdapterError> {
        let output = self.run_success(&[
            "--no-replace-objects".to_string(),
            "rev-parse".to_string(),
            "--show-object-format".to_string(),
        ])?;
        let expected = match expected {
            GitObjectAlgorithm::Sha1 => b"sha1".as_slice(),
            GitObjectAlgorithm::Sha256 => b"sha256".as_slice(),
        };
        if trim_ascii(&output.stdout) != expected {
            return Err(GitSourceAdapterError::ObjectFormatMismatch);
        }
        Ok(())
    }

    fn require_stdout(
        &self,
        args: &[&str],
        expected: &str,
        mismatch: GitSourceAdapterError,
    ) -> Result<(), GitSourceAdapterError> {
        let args = args.iter().map(|value| (*value).to_owned()).collect::<Vec<_>>();
        let output = self.run_success(&args)?;
        if trim_ascii(&output.stdout) != expected.as_bytes() {
            return Err(mismatch);
        }
        Ok(())
    }

    fn run_success(&self, args: &[String]) -> Result<GitCommandResult, GitSourceAdapterError> {
        let output = self.runner.run(&self.repository, args)?;
        if output.status != 0 {
            return Err(GitSourceAdapterError::GitCommandFailed {
                args: args.to_vec(),
                status: output.status,
            });
        }
        Ok(output)
    }
}

impl<R> ProposalSourceVerifier for GitProposalSourceAdapter<R>
where
    R: GitCommandRunner,
{
    fn identity(&self) -> SourceVerifierIdentity {
        self.identity.clone()
    }

    fn verify_source(
        &self,
        proposal: &ChangeProposal,
        _request: &RepositoryVerificationRequest,
        observation: &ProposalSourceObservation,
    ) -> Result<Digest, SourceVerifierError> {
        let inspected = self.inspect(proposal).map_err(|_| SourceVerifierError::Rejected)?;
        if observation.source_state() != &inspected.source_state
            || observation.observed_base() != proposal.base_revision()
            || observation.observed_proposed() != proposal.proposed_revision()
            || observation.observed_tree() != &inspected.tree
            || observation.ancestry_evidence() != &inspected.ancestry_evidence
            || observation.commit_tree_evidence() != &inspected.tree_evidence
        {
            return Err(SourceVerifierError::Rejected);
        }

        Ok(verifier_evidence(
            &self.expected_git_version,
            &inspected.source_state,
            &inspected.ancestry_evidence,
            &inspected.tree_evidence,
        ))
    }
}

#[derive(Clone, Debug)]
struct InspectedSource {
    tree: GitObjectId,
    source_state: Digest,
    ancestry_evidence: Digest,
    tree_evidence: Digest,
}

/// Concrete adapter failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum GitSourceAdapterError {
    /// Git executable path was not absolute.
    #[error("Git executable path must be absolute")]
    GitExecutableNotAbsolute,
    /// Configured Git version was empty or oversized.
    #[error("invalid expected Git version")]
    InvalidGitVersion,
    /// Process execution failed.
    #[error("Git process failed: {0}")]
    Process(String),
    /// Git command exited unsuccessfully.
    #[error("Git command failed with status {status}: {args:?}")]
    GitCommandFailed {
        /// Exact argv passed after `git -C <repo>`.
        args: Vec<String>,
        /// Process exit status.
        status: i32,
    },
    /// Runtime Git version differed from configured adapter identity.
    #[error("runtime Git version does not match pinned adapter version")]
    GitVersionMismatch,
    /// Repository object format differs from the proposal's object format.
    #[error("repository Git object format does not match proposal")]
    ObjectFormatMismatch,
    /// Base object is not a commit.
    #[error("proposal base object is not a commit")]
    BaseNotCommit,
    /// Proposed object is not a commit.
    #[error("proposal proposed object is not a commit")]
    ProposedNotCommit,
    /// Claimed resulting object is not a tree.
    #[error("proposal resulting-tree object is not a tree")]
    ResultNotTree,
    /// Proposed commit references another tree.
    #[error("proposed commit tree does not match proposal resulting tree")]
    TreeMismatch,
    /// Base is not an ancestor of proposed under the inspected object graph.
    #[error("proposal base is not an ancestor of proposed commit")]
    BaseNotAncestor,
    /// Underlying Forge protocol construction failed.
    #[error("Forge source protocol failure: {0}")]
    Protocol(String),
}

fn relevant_source_state(base: &GitObjectId, proposed: &GitObjectId, tree: &GitObjectId) -> Digest {
    let mut out = Vec::new();
    out.extend_from_slice(SOURCE_STATE_DOMAIN_V1);
    push_object_bytes(&mut out, base);
    push_object_bytes(&mut out, proposed);
    push_object_bytes(&mut out, tree);
    Digest::of_bytes(DigestAlgorithm::Sha256, &out)
}

fn ancestry_evidence(base: &GitObjectId, proposed: &GitObjectId) -> Digest {
    let mut out = Vec::new();
    out.extend_from_slice(ANCESTRY_EVIDENCE_DOMAIN_V1);
    push_object_bytes(&mut out, base);
    push_object_bytes(&mut out, proposed);
    Digest::of_bytes(DigestAlgorithm::Sha256, &out)
}

fn tree_evidence(proposed: &GitObjectId, tree: &GitObjectId) -> Digest {
    let mut out = Vec::new();
    out.extend_from_slice(TREE_EVIDENCE_DOMAIN_V1);
    push_object_bytes(&mut out, proposed);
    push_object_bytes(&mut out, tree);
    Digest::of_bytes(DigestAlgorithm::Sha256, &out)
}

fn verifier_evidence(
    git_version: &str,
    source_state: &Digest,
    ancestry: &Digest,
    tree: &Digest,
) -> Digest {
    let mut out = Vec::new();
    out.extend_from_slice(VERIFIER_EVIDENCE_DOMAIN_V1);
    let version_len = u16::try_from(git_version.len()).unwrap_or(u16::MAX);
    out.extend_from_slice(&version_len.to_be_bytes());
    out.extend_from_slice(git_version.as_bytes());
    out.extend_from_slice(source_state.as_bytes());
    out.extend_from_slice(ancestry.as_bytes());
    out.extend_from_slice(tree.as_bytes());
    Digest::of_bytes(DigestAlgorithm::Sha256, &out)
}

fn push_object_bytes(out: &mut Vec<u8>, object: &GitObjectId) {
    out.push(match object.algorithm() {
        GitObjectAlgorithm::Sha1 => 1,
        GitObjectAlgorithm::Sha256 => 2,
    });
    out.extend_from_slice(object.as_bytes());
}

fn object_hex(object: &GitObjectId) -> String {
    hex::encode(object.as_bytes())
}

fn trim_ascii(bytes: &[u8]) -> &[u8] {
    let mut start = 0;
    let mut end = bytes.len();
    while start < end && bytes[start].is_ascii_whitespace() {
        start += 1;
    }
    while end > start && bytes[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    &bytes[start..end]
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authority::PrincipalId;
    use mycelix_forge_core::{ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_repository::{RepositoryRef};
    use std::collections::VecDeque;
    use std::sync::Mutex;

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
            vec![],
        )
        .unwrap()
    }

    #[derive(Debug)]
    struct FakeRunner {
        outputs: Mutex<VecDeque<GitCommandResult>>,
    }

    impl FakeRunner {
        fn successful(proposal: &ChangeProposal) -> Self {
            let tree_hex = object_hex(proposal.resulting_tree());
            Self {
                outputs: Mutex::new(VecDeque::from([
                    result(0, b"git version 2.51.0\n"),
                    result(0, b"sha1\n"),
                    result(0, b"commit\n"),
                    result(0, b"commit\n"),
                    result(0, b"tree\n"),
                    result(0, format!("{tree_hex}\n").as_bytes()),
                    result(0, b""),
                ])),
            }
        }
    }

    impl GitCommandRunner for FakeRunner {
        fn run(
            &self,
            _repository: &Path,
            _args: &[String],
        ) -> Result<GitCommandResult, GitSourceAdapterError> {
            self.outputs
                .lock()
                .unwrap()
                .pop_front()
                .ok_or_else(|| GitSourceAdapterError::Process("unexpected command".to_owned()))
        }
    }

    fn result(status: i32, stdout: &[u8]) -> GitCommandResult {
        GitCommandResult {
            status,
            stdout: stdout.to_vec(),
            stderr: vec![],
        }
    }

    #[test]
    fn exact_git_semantics_produce_observation() {
        let proposal = proposal();
        let runner = FakeRunner::successful(&proposal);
        let adapter = GitProposalSourceAdapter::new(
            runner,
            PathBuf::from("/tmp/repository"),
            "2.51.0",
        )
        .unwrap();

        // The adapter only needs the request digest for the raw observation;
        // use a small real request fixture through the repository crate in the
        // integration contract tests. Here we exercise the semantic inspector
        // through its private path.
        let inspected = adapter.inspect(&proposal).unwrap();
        assert_eq!(inspected.tree, proposal.resulting_tree().clone());
    }

    #[test]
    fn tree_mismatch_fails_closed() {
        let proposal = proposal();
        let runner = FakeRunner {
            outputs: Mutex::new(VecDeque::from([
                result(0, b"git version 2.51.0\n"),
                result(0, b"sha1\n"),
                result(0, b"commit\n"),
                result(0, b"commit\n"),
                result(0, b"tree\n"),
                result(0, b"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"),
            ])),
        };
        let adapter = GitProposalSourceAdapter::new(
            runner,
            PathBuf::from("/tmp/repository"),
            "2.51.0",
        )
        .unwrap();
        assert_eq!(adapter.inspect(&proposal).unwrap_err(), GitSourceAdapterError::TreeMismatch);
    }

    #[test]
    fn non_ancestor_fails_closed() {
        let proposal = proposal();
        let tree_hex = object_hex(proposal.resulting_tree());
        let runner = FakeRunner {
            outputs: Mutex::new(VecDeque::from([
                result(0, b"git version 2.51.0\n"),
                result(0, b"sha1\n"),
                result(0, b"commit\n"),
                result(0, b"commit\n"),
                result(0, b"tree\n"),
                result(0, format!("{tree_hex}\n").as_bytes()),
                result(1, b""),
            ])),
        };
        let adapter = GitProposalSourceAdapter::new(
            runner,
            PathBuf::from("/tmp/repository"),
            "2.51.0",
        )
        .unwrap();
        assert_eq!(adapter.inspect(&proposal).unwrap_err(), GitSourceAdapterError::BaseNotAncestor);
    }

    #[test]
    fn process_runner_requires_absolute_git_path() {
        assert_eq!(
            ProcessGitRunner::new(PathBuf::from("git")).unwrap_err(),
            GitSourceAdapterError::GitExecutableNotAbsolute
        );
    }
}

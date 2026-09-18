// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Version-bound process adapter from gittuf v0.16.0 into Mycelix Forge.
//!
//! The adapter consumes only `gittuf verify-ref` exit status and raw Git ref
//! identities. Human-formatted verification output is never parsed. Portable
//! offline replay is deliberately deferred to FORGE-004C.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_repository::{
    qualify_evidence_backed_observation, AdapterIdentity, AdapterObservation, AdapterOutcome,
    EvidenceBackedAdapterObservation, GitObjectAlgorithm, GitObjectId,
    QualifiedRepositoryVerification, RepositoryEvidenceError, RepositoryPolicyState, RepositoryRef,
    RepositoryVerificationError, RepositoryVerificationRequest, VerificationCapability,
    VerificationProfile,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::{
    path::{Path, PathBuf},
    process::Command,
};
use thiserror::Error;

pub const REQUIRED_GITTUF_VERSION: &str = "0.16.0";
pub const RSL_REF: &str = "refs/gittuf/reference-state-log";
pub const POLICY_REF: &str = "refs/gittuf/policy";
pub const ATTESTATIONS_REF: &str = "refs/gittuf/attestations";
pub const PERSISTENT_CACHE_REF: &str = "refs/local/gittuf/persistent-cache";

const RECEIPT_SCHEMA_VERSION: u16 = 1;
const POLICY_SUBJECT_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-policy-subject/v1\0";
const HISTORY_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-history/v1\0";
const POLICY_LINEAGE_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-policy-lineage/v1\0";
const RECEIPT_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-local-receipt/v1\0";

/// Runtime-only invocation derived from an exact Forge request and the full
/// translated repository-policy state it names.
///
/// This type intentionally has no serde implementation: callers cannot load a
/// forged invocation from the wire and bypass [`GittufInvocation::from_request`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GittufInvocation {
    request_digest: Digest,
    policy_state: RepositoryPolicyState,
    policy_state_digest: Digest,
    reference: RepositoryRef,
    expected_tip: GitObjectId,
}

impl GittufInvocation {
    pub fn from_request(
        request: &RepositoryVerificationRequest,
        policy_state: &RepositoryPolicyState,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, AdapterError> {
        let policy_state_digest = policy_state.digest(commitment_algorithm)?;
        if &policy_state_digest != request.repository_policy_state() {
            return Err(AdapterError::RepositoryPolicyStateMismatch);
        }
        if policy_state.sequence() != request.repository_policy_sequence() {
            return Err(AdapterError::RepositoryPolicySequenceMismatch {
                request: request.repository_policy_sequence(),
                supplied: policy_state.sequence(),
            });
        }

        Ok(Self {
            request_digest: request.digest(commitment_algorithm)?,
            policy_state: policy_state.clone(),
            policy_state_digest,
            reference: request.reference().clone(),
            expected_tip: request.to().clone(),
        })
    }

    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn reference(&self) -> &RepositoryRef {
        &self.reference
    }

    pub fn expected_tip(&self) -> &GitObjectId {
        &self.expected_tip
    }
}

/// Exact local repository roots surrounding a gittuf verification operation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GittufRepositorySnapshot {
    object_format: GitObjectAlgorithm,
    protected_tip: GitObjectId,
    rsl_tip: GitObjectId,
    policy_tip: GitObjectId,
    attestations_tip: Option<GitObjectId>,
}

impl GittufRepositorySnapshot {
    pub const fn object_format(&self) -> GitObjectAlgorithm {
        self.object_format
    }

    pub fn protected_tip(&self) -> &GitObjectId {
        &self.protected_tip
    }

    pub fn rsl_tip(&self) -> &GitObjectId {
        &self.rsl_tip
    }

    pub fn policy_tip(&self) -> &GitObjectId {
        &self.policy_tip
    }

    pub fn attestations_tip(&self) -> Option<&GitObjectId> {
        self.attestations_tip.as_ref()
    }
}

/// Successful local gittuf verification receipt.
///
/// The full translated [`RepositoryPolicyState`] is retained so receipt
/// deserialization can re-derive both its state digest and its adapter-defined
/// policy subject. Filesystem paths are intentionally absent.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct GittufLocalReceipt {
    schema_version: u16,
    adapter_name: String,
    adapter_version: String,
    request_digest: Digest,
    policy_state: RepositoryPolicyState,
    policy_state_digest: Digest,
    reference: RepositoryRef,
    snapshot: GittufRepositorySnapshot,
    history_commitment: Digest,
    policy_lineage_commitment: Digest,
}

impl GittufLocalReceipt {
    fn new(
        invocation: &GittufInvocation,
        snapshot: GittufRepositorySnapshot,
    ) -> Result<Self, AdapterError> {
        validate_snapshot_algorithms(&snapshot)?;
        if invocation.expected_tip != snapshot.protected_tip {
            return Err(AdapterError::VerifiedTipMismatch);
        }

        validate_policy_mapping(&invocation.policy_state, &snapshot)?;

        let history_commitment = gittuf_history_commitment(
            snapshot.object_format,
            &snapshot.rsl_tip,
            DigestAlgorithm::Sha256,
        )?;
        let policy_lineage_commitment = gittuf_policy_lineage_commitment(
            snapshot.object_format,
            &snapshot.rsl_tip,
            &snapshot.policy_tip,
            invocation.policy_state.sequence(),
            DigestAlgorithm::Sha256,
        )?;

        let receipt = Self {
            schema_version: RECEIPT_SCHEMA_VERSION,
            adapter_name: "gittuf".to_owned(),
            adapter_version: REQUIRED_GITTUF_VERSION.to_owned(),
            request_digest: invocation.request_digest.clone(),
            policy_state: invocation.policy_state.clone(),
            policy_state_digest: invocation.policy_state_digest.clone(),
            reference: invocation.reference.clone(),
            snapshot,
            history_commitment,
            policy_lineage_commitment,
        };
        receipt.validate_internal_commitments()?;
        Ok(receipt)
    }

    pub fn snapshot(&self) -> &GittufRepositorySnapshot {
        &self.snapshot
    }

    pub fn policy_state(&self) -> &RepositoryPolicyState {
        &self.policy_state
    }

    pub fn history_commitment(&self) -> &Digest {
        &self.history_commitment
    }

    pub fn policy_lineage_commitment(&self) -> &Digest {
        &self.policy_lineage_commitment
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AdapterError> {
        self.validate_internal_commitments()?;
        let mut out = Vec::new();
        out.extend_from_slice(RECEIPT_DOMAIN_V1);
        out.extend_from_slice(&self.schema_version.to_be_bytes());
        push_string(&mut out, &self.adapter_name, "adapter_name")?;
        push_string(&mut out, &self.adapter_version, "adapter_version")?;
        push_digest(&mut out, &self.request_digest)?;
        push_digest(&mut out, &self.policy_state_digest)?;
        out.extend_from_slice(&self.policy_state.sequence().to_be_bytes());
        push_digest(&mut out, self.policy_state.policy_digest())?;
        push_string(&mut out, self.reference.as_str(), "reference")?;
        push_snapshot(&mut out, &self.snapshot)?;
        push_digest(&mut out, &self.history_commitment)?;
        push_digest(&mut out, &self.policy_lineage_commitment)?;
        Ok(out)
    }

    pub fn commitment(&self, algorithm: DigestAlgorithm) -> Result<Digest, AdapterError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }

    /// Convert this receipt into Forge evidence only in the context of the
    /// exact request it claims to verify.
    pub fn into_observation_for(
        self,
        request: &RepositoryVerificationRequest,
    ) -> Result<EvidenceBackedAdapterObservation, AdapterError> {
        self.validate_against_request(request)?;
        let observation = AdapterObservation::new(
            AdapterIdentity::new(self.adapter_name, self.adapter_version)?,
            self.request_digest,
            self.snapshot.protected_tip,
            self.policy_state_digest,
            Some(self.history_commitment),
            None,
            local_full_capabilities(),
            AdapterOutcome::Verified,
        )?;
        Ok(EvidenceBackedAdapterObservation::new(
            observation,
            Some(self.policy_lineage_commitment),
        )?)
    }

    fn validate_against_request(
        &self,
        request: &RepositoryVerificationRequest,
    ) -> Result<(), AdapterError> {
        self.validate_internal_commitments()?;
        let expected_request = request.digest(self.request_digest.algorithm())?;
        if expected_request != self.request_digest {
            return Err(AdapterError::ReceiptRequestMismatch);
        }
        if request.repository_policy_state() != &self.policy_state_digest {
            return Err(AdapterError::RepositoryPolicyStateMismatch);
        }
        if request.repository_policy_sequence() != self.policy_state.sequence() {
            return Err(AdapterError::RepositoryPolicySequenceMismatch {
                request: request.repository_policy_sequence(),
                supplied: self.policy_state.sequence(),
            });
        }
        if request.reference() != &self.reference {
            return Err(AdapterError::ReceiptReferenceMismatch);
        }
        if request.to() != &self.snapshot.protected_tip {
            return Err(AdapterError::VerifiedTipMismatch);
        }
        Ok(())
    }

    fn validate_internal_commitments(&self) -> Result<(), AdapterError> {
        if self.schema_version != RECEIPT_SCHEMA_VERSION {
            return Err(AdapterError::UnsupportedReceiptSchema(self.schema_version));
        }
        if self.adapter_name != "gittuf" || self.adapter_version != REQUIRED_GITTUF_VERSION {
            return Err(AdapterError::UnexpectedReceiptAdapter {
                name: self.adapter_name.clone(),
                version: self.adapter_version.clone(),
            });
        }
        validate_snapshot_algorithms(&self.snapshot)?;

        let expected_policy_state = self
            .policy_state
            .digest(self.policy_state_digest.algorithm())?;
        if expected_policy_state != self.policy_state_digest {
            return Err(AdapterError::ReceiptPolicyStateMismatch);
        }
        validate_policy_mapping(&self.policy_state, &self.snapshot)?;

        let expected_history = gittuf_history_commitment(
            self.snapshot.object_format,
            &self.snapshot.rsl_tip,
            self.history_commitment.algorithm(),
        )?;
        if expected_history != self.history_commitment {
            return Err(AdapterError::HistoryCommitmentMismatch);
        }

        let expected_lineage = gittuf_policy_lineage_commitment(
            self.snapshot.object_format,
            &self.snapshot.rsl_tip,
            &self.snapshot.policy_tip,
            self.policy_state.sequence(),
            self.policy_lineage_commitment.algorithm(),
        )?;
        if expected_lineage != self.policy_lineage_commitment {
            return Err(AdapterError::PolicyLineageCommitmentMismatch);
        }
        Ok(())
    }
}

impl<'de> Deserialize<'de> for GittufLocalReceipt {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireReceipt {
            schema_version: u16,
            adapter_name: String,
            adapter_version: String,
            request_digest: Digest,
            policy_state: RepositoryPolicyState,
            policy_state_digest: Digest,
            reference: RepositoryRef,
            snapshot: GittufRepositorySnapshot,
            history_commitment: Digest,
            policy_lineage_commitment: Digest,
        }

        let wire = WireReceipt::deserialize(deserializer)?;
        let receipt = Self {
            schema_version: wire.schema_version,
            adapter_name: wire.adapter_name,
            adapter_version: wire.adapter_version,
            request_digest: wire.request_digest,
            policy_state: wire.policy_state,
            policy_state_digest: wire.policy_state_digest,
            reference: wire.reference,
            snapshot: wire.snapshot,
            history_commitment: wire.history_commitment,
            policy_lineage_commitment: wire.policy_lineage_commitment,
        };
        receipt.validate_internal_commitments().map_err(D::Error::custom)?;
        Ok(receipt)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandSpec {
    pub program: PathBuf,
    pub args: Vec<String>,
    pub cwd: PathBuf,
    pub env: Vec<(String, String)>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandOutput {
    pub status: i32,
    pub stdout: String,
    pub stderr: String,
}

pub trait CommandRunner {
    fn run(&self, spec: &CommandSpec) -> Result<CommandOutput, AdapterError>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SystemCommandRunner;

impl CommandRunner for SystemCommandRunner {
    fn run(&self, spec: &CommandSpec) -> Result<CommandOutput, AdapterError> {
        let output = Command::new(&spec.program)
            .args(&spec.args)
            .current_dir(&spec.cwd)
            .envs(spec.env.iter().map(|(key, value)| (key, value)))
            .output()
            .map_err(|source| AdapterError::SpawnCommand {
                program: spec.program.clone(),
                source,
            })?;
        let stdout = String::from_utf8(output.stdout).map_err(|source| {
            AdapterError::NonUtf8Output {
                program: spec.program.clone(),
                stream: "stdout",
                source,
            }
        })?;
        let stderr = String::from_utf8(output.stderr).map_err(|source| {
            AdapterError::NonUtf8Output {
                program: spec.program.clone(),
                stream: "stderr",
                source,
            }
        })?;
        Ok(CommandOutput {
            status: output.status.code().unwrap_or(-1),
            stdout,
            stderr,
        })
    }
}

pub struct GittufAdapter<R> {
    runner: R,
    gittuf_binary: PathBuf,
    git_binary: PathBuf,
}

impl GittufAdapter<SystemCommandRunner> {
    pub fn system() -> Self {
        Self::new(SystemCommandRunner, "gittuf", "git")
    }
}

impl<R: CommandRunner> GittufAdapter<R> {
    pub fn new(
        runner: R,
        gittuf_binary: impl Into<PathBuf>,
        git_binary: impl Into<PathBuf>,
    ) -> Self {
        Self {
            runner,
            gittuf_binary: gittuf_binary.into(),
            git_binary: git_binary.into(),
        }
    }

    /// Execute local from-first-RSL-entry verification without a network fetch.
    ///
    /// gittuf's local persistent cache is rejected because v0.16 may otherwise
    /// resume VerifyRefFull from the cache's last-verified entry. Mycelix's
    /// `FullHistory` capability intentionally does not inherit that hidden
    /// local trust input.
    pub fn observe(
        &self,
        repository_path: &Path,
        invocation: &GittufInvocation,
    ) -> Result<GittufLocalReceipt, AdapterError> {
        self.verify_gittuf_version(repository_path)?;
        let object_format = self.read_object_format(repository_path)?;
        if invocation.expected_tip.algorithm() != object_format {
            return Err(AdapterError::ObjectFormatMismatch {
                repository: object_format,
                object: invocation.expected_tip.algorithm(),
            });
        }

        self.ensure_persistent_cache_absent(repository_path)?;
        let before = self.read_snapshot(repository_path, &invocation.reference, object_format)?;
        if before.protected_tip != invocation.expected_tip {
            return Err(AdapterError::VerifiedTipMismatch);
        }
        validate_policy_mapping(&invocation.policy_state, &before)?;

        // v0.16 defaults to VerifyRef's full-history path. Never add
        // `--latest-only` or developer-only `--from-entry` here.
        let verify = self.run(
            &self.gittuf_binary,
            vec!["verify-ref".to_owned(), invocation.reference.to_string()],
            repository_path,
        )?;
        require_success(
            &self.gittuf_binary,
            &["verify-ref", invocation.reference.as_str()],
            &verify,
        )?;

        self.ensure_persistent_cache_absent(repository_path)?;
        let after = self.read_snapshot(repository_path, &invocation.reference, object_format)?;
        if before != after {
            return Err(AdapterError::RepositoryChangedDuringVerification);
        }

        GittufLocalReceipt::new(invocation, after)
    }

    pub fn verify_request(
        &self,
        repository_path: &Path,
        request: &RepositoryVerificationRequest,
        policy_state: &RepositoryPolicyState,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<QualifiedRepositoryVerification, AdapterError> {
        let invocation =
            GittufInvocation::from_request(request, policy_state, commitment_algorithm)?;
        let receipt = self.observe(repository_path, &invocation)?;
        let observation = receipt.into_observation_for(request)?;
        Ok(qualify_evidence_backed_observation(
            &gittuf_local_full_profile(),
            request,
            observation,
        )?)
    }

    fn verify_gittuf_version(&self, cwd: &Path) -> Result<(), AdapterError> {
        let output = self.run(&self.gittuf_binary, vec!["version".to_owned()], cwd)?;
        require_success(&self.gittuf_binary, &["version"], &output)?;
        let expected = format!("gittuf version {REQUIRED_GITTUF_VERSION}");
        if output.stdout.trim() != expected {
            return Err(AdapterError::UnsupportedGittufVersion {
                expected: REQUIRED_GITTUF_VERSION.to_owned(),
                actual: output.stdout.trim().to_owned(),
            });
        }
        Ok(())
    }

    fn read_object_format(&self, cwd: &Path) -> Result<GitObjectAlgorithm, AdapterError> {
        let output = self.run(
            &self.git_binary,
            vec!["rev-parse".to_owned(), "--show-object-format".to_owned()],
            cwd,
        )?;
        require_success(&self.git_binary, &["rev-parse", "--show-object-format"], &output)?;
        match output.stdout.trim() {
            "sha1" => Ok(GitObjectAlgorithm::Sha1),
            "sha256" => Ok(GitObjectAlgorithm::Sha256),
            other => Err(AdapterError::UnsupportedGitObjectFormat(other.to_owned())),
        }
    }

    fn ensure_persistent_cache_absent(&self, cwd: &Path) -> Result<(), AdapterError> {
        let output = self.run(
            &self.git_binary,
            vec![
                "rev-parse".to_owned(),
                "--verify".to_owned(),
                "--quiet".to_owned(),
                PERSISTENT_CACHE_REF.to_owned(),
            ],
            cwd,
        )?;
        match output.status {
            1 if output.stdout.trim().is_empty() => Ok(()),
            0 => Err(AdapterError::PersistentCachePresent),
            _ => Err(AdapterError::CommandFailed {
                program: self.git_binary.clone(),
                args: vec![
                    "rev-parse".to_owned(),
                    "--verify".to_owned(),
                    "--quiet".to_owned(),
                    PERSISTENT_CACHE_REF.to_owned(),
                ],
                status: output.status,
                stderr: output.stderr,
            }),
        }
    }

    fn read_snapshot(
        &self,
        cwd: &Path,
        reference: &RepositoryRef,
        object_format: GitObjectAlgorithm,
    ) -> Result<GittufRepositorySnapshot, AdapterError> {
        let snapshot = GittufRepositorySnapshot {
            object_format,
            protected_tip: self.read_required_ref(cwd, reference.as_str(), object_format)?,
            rsl_tip: self.read_required_ref(cwd, RSL_REF, object_format)?,
            policy_tip: self.read_required_ref(cwd, POLICY_REF, object_format)?,
            attestations_tip: self.read_optional_ref(cwd, ATTESTATIONS_REF, object_format)?,
        };
        validate_snapshot_algorithms(&snapshot)?;
        Ok(snapshot)
    }

    fn read_required_ref(
        &self,
        cwd: &Path,
        reference: &str,
        object_format: GitObjectAlgorithm,
    ) -> Result<GitObjectId, AdapterError> {
        let output = self.run(
            &self.git_binary,
            vec![
                "rev-parse".to_owned(),
                "--verify".to_owned(),
                reference.to_owned(),
            ],
            cwd,
        )?;
        require_success(&self.git_binary, &["rev-parse", "--verify", reference], &output)?;
        parse_git_object_id(output.stdout.trim(), object_format)
    }

    fn read_optional_ref(
        &self,
        cwd: &Path,
        reference: &str,
        object_format: GitObjectAlgorithm,
    ) -> Result<Option<GitObjectId>, AdapterError> {
        let output = self.run(
            &self.git_binary,
            vec![
                "rev-parse".to_owned(),
                "--verify".to_owned(),
                "--quiet".to_owned(),
                reference.to_owned(),
            ],
            cwd,
        )?;
        match output.status {
            0 => Ok(Some(parse_git_object_id(
                output.stdout.trim(),
                object_format,
            )?)),
            1 if output.stdout.trim().is_empty() => Ok(None),
            _ => Err(AdapterError::CommandFailed {
                program: self.git_binary.clone(),
                args: vec![
                    "rev-parse".to_owned(),
                    "--verify".to_owned(),
                    "--quiet".to_owned(),
                    reference.to_owned(),
                ],
                status: output.status,
                stderr: output.stderr,
            }),
        }
    }

    fn run(
        &self,
        program: &Path,
        args: Vec<String>,
        cwd: &Path,
    ) -> Result<CommandOutput, AdapterError> {
        self.runner.run(&CommandSpec {
            program: program.to_path_buf(),
            args,
            cwd: cwd.to_path_buf(),
            env: vec![
                ("GITTUF_DEV".to_owned(), "0".to_owned()),
                ("GITTUF_DEBUG".to_owned(), "0".to_owned()),
                ("LC_ALL".to_owned(), "C".to_owned()),
                ("LANG".to_owned(), "C".to_owned()),
            ],
        })
    }
}

pub fn gittuf_local_full_profile() -> VerificationProfile {
    VerificationProfile::new(local_full_capabilities()).expect("static profile is non-empty")
}

fn local_full_capabilities() -> [VerificationCapability; 4] {
    [
        VerificationCapability::RefTipBinding,
        VerificationCapability::FullHistory,
        VerificationCapability::ProtectedRewriteDetection,
        VerificationCapability::PolicyLineageMonotonic,
    ]
}

/// Adapter-defined identity for one active gittuf policy ref tip.
pub fn gittuf_policy_subject_commitment(
    object_format: GitObjectAlgorithm,
    policy_tip: &GitObjectId,
    algorithm: DigestAlgorithm,
) -> Result<Digest, AdapterError> {
    ensure_object_algorithm(object_format, policy_tip)?;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(POLICY_SUBJECT_DOMAIN_V1);
    push_git_object(&mut bytes, policy_tip)?;
    Ok(Digest::of_bytes(algorithm, &bytes))
}

pub fn gittuf_history_commitment(
    object_format: GitObjectAlgorithm,
    rsl_tip: &GitObjectId,
    algorithm: DigestAlgorithm,
) -> Result<Digest, AdapterError> {
    ensure_object_algorithm(object_format, rsl_tip)?;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(HISTORY_DOMAIN_V1);
    push_git_object(&mut bytes, rsl_tip)?;
    Ok(Digest::of_bytes(algorithm, &bytes))
}

pub fn gittuf_policy_lineage_commitment(
    object_format: GitObjectAlgorithm,
    rsl_tip: &GitObjectId,
    policy_tip: &GitObjectId,
    repository_policy_sequence: u64,
    algorithm: DigestAlgorithm,
) -> Result<Digest, AdapterError> {
    ensure_object_algorithm(object_format, rsl_tip)?;
    ensure_object_algorithm(object_format, policy_tip)?;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(POLICY_LINEAGE_DOMAIN_V1);
    push_git_object(&mut bytes, rsl_tip)?;
    push_git_object(&mut bytes, policy_tip)?;
    bytes.extend_from_slice(&repository_policy_sequence.to_be_bytes());
    Ok(Digest::of_bytes(algorithm, &bytes))
}

fn validate_policy_mapping(
    policy_state: &RepositoryPolicyState,
    snapshot: &GittufRepositorySnapshot,
) -> Result<(), AdapterError> {
    let expected = gittuf_policy_subject_commitment(
        snapshot.object_format,
        &snapshot.policy_tip,
        policy_state.policy_digest().algorithm(),
    )?;
    if &expected != policy_state.policy_digest() {
        return Err(AdapterError::ExternalPolicyCommitmentMismatch);
    }
    Ok(())
}

fn parse_git_object_id(
    value: &str,
    algorithm: GitObjectAlgorithm,
) -> Result<GitObjectId, AdapterError> {
    let bytes =
        hex::decode(value).map_err(|_| AdapterError::InvalidGitObjectHex(value.to_owned()))?;
    Ok(GitObjectId::new(algorithm, bytes)?)
}

fn validate_snapshot_algorithms(snapshot: &GittufRepositorySnapshot) -> Result<(), AdapterError> {
    ensure_object_algorithm(snapshot.object_format, &snapshot.protected_tip)?;
    ensure_object_algorithm(snapshot.object_format, &snapshot.rsl_tip)?;
    ensure_object_algorithm(snapshot.object_format, &snapshot.policy_tip)?;
    if let Some(attestations) = &snapshot.attestations_tip {
        ensure_object_algorithm(snapshot.object_format, attestations)?;
    }
    Ok(())
}

fn ensure_object_algorithm(
    algorithm: GitObjectAlgorithm,
    object: &GitObjectId,
) -> Result<(), AdapterError> {
    if object.algorithm() == algorithm {
        Ok(())
    } else {
        Err(AdapterError::ObjectFormatMismatch {
            repository: algorithm,
            object: object.algorithm(),
        })
    }
}

fn require_success(
    program: &Path,
    args: &[&str],
    output: &CommandOutput,
) -> Result<(), AdapterError> {
    if output.status == 0 {
        Ok(())
    } else {
        Err(AdapterError::CommandFailed {
            program: program.to_path_buf(),
            args: args.iter().map(|arg| (*arg).to_owned()).collect(),
            status: output.status,
            stderr: output.stderr.clone(),
        })
    }
}

fn push_snapshot(
    out: &mut Vec<u8>,
    snapshot: &GittufRepositorySnapshot,
) -> Result<(), AdapterError> {
    validate_snapshot_algorithms(snapshot)?;
    out.push(match snapshot.object_format {
        GitObjectAlgorithm::Sha1 => 1,
        GitObjectAlgorithm::Sha256 => 2,
    });
    push_git_object(out, &snapshot.protected_tip)?;
    push_git_object(out, &snapshot.rsl_tip)?;
    push_git_object(out, &snapshot.policy_tip)?;
    match &snapshot.attestations_tip {
        Some(tip) => {
            out.push(1);
            push_git_object(out, tip)?;
        }
        None => out.push(0),
    }
    Ok(())
}

fn push_git_object(out: &mut Vec<u8>, object: &GitObjectId) -> Result<(), AdapterError> {
    out.push(match object.algorithm() {
        GitObjectAlgorithm::Sha1 => 1,
        GitObjectAlgorithm::Sha256 => 2,
    });
    let len = u16::try_from(object.as_bytes().len())
        .map_err(|_| AdapterError::CanonicalFieldTooLarge("git_object_id"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(object.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), AdapterError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| AdapterError::CanonicalFieldTooLarge("digest_algorithm"))?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    let digest_len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| AdapterError::CanonicalFieldTooLarge("digest_bytes"))?;
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(out: &mut Vec<u8>, value: &str, field: &'static str) -> Result<(), AdapterError> {
    let bytes = value.as_bytes();
    let len =
        u16::try_from(bytes.len()).map_err(|_| AdapterError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

#[derive(Debug, Error)]
pub enum AdapterError {
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error(transparent)]
    Evidence(#[from] RepositoryEvidenceError),
    #[error("supplied repository-policy state does not match the exact verification request")]
    RepositoryPolicyStateMismatch,
    #[error("repository-policy sequence mismatch: request={request}, supplied={supplied}")]
    RepositoryPolicySequenceMismatch { request: u64, supplied: u64 },
    #[error("unsupported gittuf version: expected {expected}, got {actual}")]
    UnsupportedGittufVersion { expected: String, actual: String },
    #[error("unsupported Git object format: {0}")]
    UnsupportedGitObjectFormat(String),
    #[error("repository object format {repository:?} does not match object format {object:?}")]
    ObjectFormatMismatch {
        repository: GitObjectAlgorithm,
        object: GitObjectAlgorithm,
    },
    #[error("gittuf persistent cache is present; FullHistory qualification requires cache-free verification")]
    PersistentCachePresent,
    #[error("protected ref tip does not match the exact verification request")]
    VerifiedTipMismatch,
    #[error("gittuf policy ref does not match the translated repository-policy subject")]
    ExternalPolicyCommitmentMismatch,
    #[error("repository refs changed while gittuf verification was executing")]
    RepositoryChangedDuringVerification,
    #[error("receipt policy state digest does not match its embedded policy state")]
    ReceiptPolicyStateMismatch,
    #[error("receipt does not bind the exact verification request")]
    ReceiptRequestMismatch,
    #[error("receipt reference does not match the exact verification request")]
    ReceiptReferenceMismatch,
    #[error("invalid Git object hex: {0}")]
    InvalidGitObjectHex(String),
    #[error("unsupported receipt schema: {0}")]
    UnsupportedReceiptSchema(u16),
    #[error("unexpected receipt adapter identity: {name} {version}")]
    UnexpectedReceiptAdapter { name: String, version: String },
    #[error("receipt history commitment does not match its RSL tip")]
    HistoryCommitmentMismatch,
    #[error("receipt policy-lineage commitment does not match its metadata roots")]
    PolicyLineageCommitmentMismatch,
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
    #[error("failed to spawn {program:?}: {source}")]
    SpawnCommand {
        program: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("{program:?} emitted non-UTF8 {stream}: {source}")]
    NonUtf8Output {
        program: PathBuf,
        stream: &'static str,
        #[source]
        source: std::string::FromUtf8Error,
    },
    #[error("command failed: {program:?} {args:?}, status={status}, stderr={stderr}")]
    CommandFailed {
        program: PathBuf,
        args: Vec<String>,
        status: i32,
        stderr: String,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_repository::{RepositoryAdoption, RepositoryTip};
    use std::{cell::RefCell, collections::VecDeque};

    #[derive(Clone)]
    struct ExpectedCall {
        program: PathBuf,
        args: Vec<String>,
        cwd: PathBuf,
        output: CommandOutput,
    }

    struct FakeRunner {
        calls: RefCell<VecDeque<ExpectedCall>>,
    }

    impl FakeRunner {
        fn new(calls: Vec<ExpectedCall>) -> Self {
            Self {
                calls: RefCell::new(calls.into()),
            }
        }

        fn remaining(&self) -> usize {
            self.calls.borrow().len()
        }
    }

    impl CommandRunner for FakeRunner {
        fn run(&self, spec: &CommandSpec) -> Result<CommandOutput, AdapterError> {
            let expected = self
                .calls
                .borrow_mut()
                .pop_front()
                .expect("unexpected command");
            assert_eq!(spec.program, expected.program);
            assert_eq!(spec.args, expected.args);
            assert_eq!(spec.cwd, expected.cwd);
            assert!(spec
                .env
                .contains(&("GITTUF_DEV".to_owned(), "0".to_owned())));
            assert!(!spec.args.iter().any(|arg| arg == "--latest-only"));
            assert!(!spec.args.iter().any(|arg| arg == "--from-entry"));
            Ok(expected.output)
        }
    }

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn git_id(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn hex_id(byte: u8) -> String {
        hex::encode(vec![byte; 20])
    }

    fn request_and_state() -> (RepositoryVerificationRequest, RepositoryPolicyState) {
        let policy_tip = git_id(0x99);
        let policy_commitment = gittuf_policy_subject_commitment(
            GitObjectAlgorithm::Sha1,
            &policy_tip,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let state =
            RepositoryPolicyState::new(project(), 0, None, policy_commitment.clone()).unwrap();
        let adoption = RepositoryAdoption::new(
            project(),
            RepositoryTip::new(
                RepositoryRef::new("refs/heads/main").unwrap(),
                git_id(0x33),
            ),
            digest(0x44),
            digest(0x55),
            policy_commitment,
            1_000,
        );
        let request = RepositoryVerificationRequest::new(
            &adoption,
            git_id(0x33),
            git_id(0x77),
            digest(0x44),
            digest(0x55),
            &state,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        (request, state)
    }

    fn ok(stdout: impl Into<String>) -> CommandOutput {
        CommandOutput {
            status: 0,
            stdout: stdout.into(),
            stderr: String::new(),
        }
    }

    fn missing_ref() -> CommandOutput {
        CommandOutput {
            status: 1,
            stdout: String::new(),
            stderr: String::new(),
        }
    }

    fn cache_check(repo: &Path, present: bool) -> ExpectedCall {
        ExpectedCall {
            program: PathBuf::from("git"),
            args: vec![
                "rev-parse".into(),
                "--verify".into(),
                "--quiet".into(),
                PERSISTENT_CACHE_REF.into(),
            ],
            cwd: repo.to_path_buf(),
            output: if present {
                ok(format!("{}\n", hex_id(0xcc)))
            } else {
                missing_ref()
            },
        }
    }

    fn snapshot_calls(repo: &Path, attestation: bool) -> Vec<ExpectedCall> {
        let mut calls = vec![
            ExpectedCall {
                program: PathBuf::from("git"),
                args: vec![
                    "rev-parse".into(),
                    "--verify".into(),
                    "refs/heads/main".into(),
                ],
                cwd: repo.to_path_buf(),
                output: ok(format!("{}\n", hex_id(0x77))),
            },
            ExpectedCall {
                program: PathBuf::from("git"),
                args: vec!["rev-parse".into(), "--verify".into(), RSL_REF.into()],
                cwd: repo.to_path_buf(),
                output: ok(format!("{}\n", hex_id(0x88))),
            },
            ExpectedCall {
                program: PathBuf::from("git"),
                args: vec!["rev-parse".into(), "--verify".into(), POLICY_REF.into()],
                cwd: repo.to_path_buf(),
                output: ok(format!("{}\n", hex_id(0x99))),
            },
        ];
        calls.push(ExpectedCall {
            program: PathBuf::from("git"),
            args: vec![
                "rev-parse".into(),
                "--verify".into(),
                "--quiet".into(),
                ATTESTATIONS_REF.into(),
            ],
            cwd: repo.to_path_buf(),
            output: if attestation {
                ok(format!("{}\n", hex_id(0xaa)))
            } else {
                missing_ref()
            },
        });
        calls
    }

    fn successful_calls(repo: &Path, attestation: bool) -> Vec<ExpectedCall> {
        let mut calls = vec![
            ExpectedCall {
                program: PathBuf::from("gittuf"),
                args: vec!["version".into()],
                cwd: repo.to_path_buf(),
                output: ok("gittuf version 0.16.0\n"),
            },
            ExpectedCall {
                program: PathBuf::from("git"),
                args: vec!["rev-parse".into(), "--show-object-format".into()],
                cwd: repo.to_path_buf(),
                output: ok("sha1\n"),
            },
            cache_check(repo, false),
        ];
        calls.extend(snapshot_calls(repo, attestation));
        calls.push(ExpectedCall {
            program: PathBuf::from("gittuf"),
            args: vec!["verify-ref".into(), "refs/heads/main".into()],
            cwd: repo.to_path_buf(),
            output: ok(""),
        });
        calls.push(cache_check(repo, false));
        calls.extend(snapshot_calls(repo, attestation));
        calls
    }

    fn receipt(repo: &Path) -> (GittufLocalReceipt, RepositoryVerificationRequest) {
        let (request, state) = request_and_state();
        let invocation =
            GittufInvocation::from_request(&request, &state, DigestAlgorithm::Sha256).unwrap();
        let receipt = GittufAdapter::new(
            FakeRunner::new(successful_calls(repo, false)),
            "gittuf",
            "git",
        )
        .observe(repo, &invocation)
        .unwrap();
        (receipt, request)
    }

    #[test]
    fn full_verification_uses_no_cache_latest_only_or_developer_mode() {
        let repo = Path::new("/tmp/repo-a");
        let runner = FakeRunner::new(successful_calls(repo, true));
        let adapter = GittufAdapter::new(runner, "gittuf", "git");
        let (request, state) = request_and_state();
        let invocation =
            GittufInvocation::from_request(&request, &state, DigestAlgorithm::Sha256).unwrap();
        let receipt = adapter.observe(repo, &invocation).unwrap();
        assert_eq!(adapter.runner.remaining(), 0);
        assert_eq!(receipt.snapshot().protected_tip(), request.to());
        assert!(receipt.snapshot().attestations_tip().is_some());
    }

    #[test]
    fn persistent_cache_fails_before_verification() {
        let repo = Path::new("/tmp/repo");
        let runner = FakeRunner::new(vec![
            ExpectedCall {
                program: PathBuf::from("gittuf"),
                args: vec!["version".into()],
                cwd: repo.to_path_buf(),
                output: ok("gittuf version 0.16.0\n"),
            },
            ExpectedCall {
                program: PathBuf::from("git"),
                args: vec!["rev-parse".into(), "--show-object-format".into()],
                cwd: repo.to_path_buf(),
                output: ok("sha1\n"),
            },
            cache_check(repo, true),
        ]);
        let adapter = GittufAdapter::new(runner, "gittuf", "git");
        let (request, state) = request_and_state();
        let invocation =
            GittufInvocation::from_request(&request, &state, DigestAlgorithm::Sha256).unwrap();
        assert!(matches!(
            adapter.observe(repo, &invocation),
            Err(AdapterError::PersistentCachePresent)
        ));
    }

    #[test]
    fn identical_repository_at_different_path_has_identical_receipt() {
        let (request, state) = request_and_state();
        let invocation =
            GittufInvocation::from_request(&request, &state, DigestAlgorithm::Sha256).unwrap();
        let repo_a = Path::new("/srv/a");
        let repo_b = Path::new("/mnt/b");
        let a = GittufAdapter::new(
            FakeRunner::new(successful_calls(repo_a, false)),
            "gittuf",
            "git",
        )
        .observe(repo_a, &invocation)
        .unwrap();
        let b = GittufAdapter::new(
            FakeRunner::new(successful_calls(repo_b, false)),
            "gittuf",
            "git",
        )
        .observe(repo_b, &invocation)
        .unwrap();
        assert_eq!(a, b);
        assert_eq!(
            a.commitment(DigestAlgorithm::Sha256).unwrap(),
            b.commitment(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn unsupported_gittuf_version_fails_before_verification() {
        let repo = Path::new("/tmp/repo");
        let runner = FakeRunner::new(vec![ExpectedCall {
            program: PathBuf::from("gittuf"),
            args: vec!["version".into()],
            cwd: repo.to_path_buf(),
            output: ok("gittuf version 0.17.0\n"),
        }]);
        let adapter = GittufAdapter::new(runner, "gittuf", "git");
        let (request, state) = request_and_state();
        let invocation =
            GittufInvocation::from_request(&request, &state, DigestAlgorithm::Sha256).unwrap();
        assert!(matches!(
            adapter.observe(repo, &invocation),
            Err(AdapterError::UnsupportedGittufVersion { .. })
        ));
    }

    #[test]
    fn repository_mutation_during_verification_fails_closed() {
        let repo = Path::new("/tmp/repo");
        let mut calls = vec![
            ExpectedCall {
                program: PathBuf::from("gittuf"),
                args: vec!["version".into()],
                cwd: repo.to_path_buf(),
                output: ok("gittuf version 0.16.0\n"),
            },
            ExpectedCall {
                program: PathBuf::from("git"),
                args: vec!["rev-parse".into(), "--show-object-format".into()],
                cwd: repo.to_path_buf(),
                output: ok("sha1\n"),
            },
            cache_check(repo, false),
        ];
        calls.extend(snapshot_calls(repo, false));
        calls.push(ExpectedCall {
            program: PathBuf::from("gittuf"),
            args: vec!["verify-ref".into(), "refs/heads/main".into()],
            cwd: repo.to_path_buf(),
            output: ok(""),
        });
        calls.push(cache_check(repo, false));
        let mut after = snapshot_calls(repo, false);
        after[1].output = ok(format!("{}\n", hex_id(0x89)));
        calls.extend(after);

        let adapter = GittufAdapter::new(FakeRunner::new(calls), "gittuf", "git");
        let (request, state) = request_and_state();
        let invocation =
            GittufInvocation::from_request(&request, &state, DigestAlgorithm::Sha256).unwrap();
        assert!(matches!(
            adapter.observe(repo, &invocation),
            Err(AdapterError::RepositoryChangedDuringVerification)
        ));
    }

    #[test]
    fn local_receipt_does_not_claim_offline_evidence() {
        let profile = gittuf_local_full_profile();
        assert!(!profile
            .required()
            .contains(&VerificationCapability::OfflineEvidence));
        assert!(profile
            .required()
            .contains(&VerificationCapability::FullHistory));
    }

    #[test]
    fn receipt_deserialization_revalidates_policy_state() {
        let (receipt, _) = receipt(Path::new("/tmp/repo"));
        let mut value = serde_json::to_value(&receipt).unwrap();
        let original = value["policy_state_digest"]["bytes"][0]
            .as_u64()
            .unwrap();
        value["policy_state_digest"]["bytes"][0] = serde_json::json!(original ^ 1);
        assert!(serde_json::from_value::<GittufLocalReceipt>(value).is_err());
    }

    #[test]
    fn receipt_deserialization_revalidates_policy_subject_mapping() {
        let (receipt, _) = receipt(Path::new("/tmp/repo"));
        let mut value = serde_json::to_value(&receipt).unwrap();
        let original = value["policy_state"]["policy_digest"]["bytes"][0]
            .as_u64()
            .unwrap();
        value["policy_state"]["policy_digest"]["bytes"][0] = serde_json::json!(original ^ 1);
        assert!(serde_json::from_value::<GittufLocalReceipt>(value).is_err());
    }

    #[test]
    fn receipt_cannot_qualify_a_different_request() {
        let (receipt, original_request) = receipt(Path::new("/tmp/repo"));
        let different = RepositoryVerificationRequest::new(
            &RepositoryAdoption::new(
                project(),
                RepositoryTip::new(
                    RepositoryRef::new("refs/heads/main").unwrap(),
                    git_id(0x33),
                ),
                digest(0x44),
                digest(0x56),
                receipt.policy_state().policy_digest().clone(),
                1_000,
            ),
            git_id(0x33),
            original_request.to().clone(),
            digest(0x44),
            digest(0x56),
            receipt.policy_state(),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert!(matches!(
            receipt.into_observation_for(&different),
            Err(AdapterError::ReceiptRequestMismatch)
        ));
    }

    #[test]
    fn exact_request_can_be_qualified_from_receipt() {
        let repo = Path::new("/tmp/repo");
        let adapter = GittufAdapter::new(
            FakeRunner::new(successful_calls(repo, false)),
            "gittuf",
            "git",
        );
        let (request, state) = request_and_state();
        let qualified = adapter
            .verify_request(repo, &request, &state, DigestAlgorithm::Sha256)
            .unwrap();
        assert_eq!(qualified.structural().observed_tip(), request.to());
        assert!(qualified.policy_lineage_commitment().is_some());
    }
}

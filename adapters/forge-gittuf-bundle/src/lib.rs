// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Self-contained Git repository closure and replay for Mycelix Forge.
//!
//! FORGE-004C proves that the protected source and required gittuf metadata can
//! be carried as a self-contained Git bundle and replayed into an empty
//! repository. It deliberately does **not** mint the protocol-level
//! `OfflineEvidence` capability: gittuf signature verification may still depend
//! on verifier-side trust material (notably Sigstore/TUF). FORGE-004D closes
//! that verifier-environment boundary.

mod manifest;

pub use manifest::{
    canonicalize_refs, BundleRef, OfflineBundleManifest, OfflineManifestError,
    OFFLINE_BUNDLE_SCHEMA_VERSION, REQUIRED_BUNDLE_FORMAT,
};

use mycelix_forge_core::{Digest, DigestAlgorithm, ForgeCoreError};
use mycelix_forge_gittuf_adapter::{
    AdapterError, CommandOutput, CommandRunner, CommandSpec, GittufAdapter, GittufInvocation,
    GittufLocalReceipt, SystemCommandRunner, ATTESTATIONS_REF, POLICY_REF, RSL_REF,
};
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    RepositoryVerificationError, RepositoryVerificationRequest,
};
use sha2::{Digest as ShaDigest, Sha256};
use std::{
    fs::{self, File},
    io::{BufRead, BufReader, Read},
    path::{Path, PathBuf},
};
use thiserror::Error;

const BUNDLE_V3_HEADER: &str = "# v3 git bundle";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArtifactFingerprint {
    pub digest: Digest,
    pub size: u64,
}

/// Positive 004C result: the exact Git closure replayed successfully and
/// produced the same FORGE-004B local receipt subject.
///
/// This is intentionally not a [`mycelix_forge_repository::QualifiedRepositoryVerification`]
/// with `OfflineEvidence`. Hermetic verifier closure is deferred to FORGE-004D.
#[derive(Clone, Debug)]
pub struct PortableRepositoryReplay {
    manifest: OfflineBundleManifest,
    replay_receipt: GittufLocalReceipt,
}

impl PortableRepositoryReplay {
    pub fn manifest(&self) -> &OfflineBundleManifest {
        &self.manifest
    }

    pub fn replay_receipt(&self) -> &GittufLocalReceipt {
        &self.replay_receipt
    }
}

pub struct OfflineBundleAdapter<R> {
    runner: R,
    git_binary: PathBuf,
    gittuf_binary: PathBuf,
}

impl OfflineBundleAdapter<SystemCommandRunner> {
    pub fn system() -> Self {
        Self::new(SystemCommandRunner, "git", "gittuf")
    }
}

impl<R: CommandRunner + Clone> OfflineBundleAdapter<R> {
    pub fn new(
        runner: R,
        git_binary: impl Into<PathBuf>,
        gittuf_binary: impl Into<PathBuf>,
    ) -> Self {
        Self {
            runner,
            git_binary: git_binary.into(),
            gittuf_binary: gittuf_binary.into(),
        }
    }

    /// Build a self-contained v3 Git closure from one stable local verification.
    pub fn create_bundle(
        &self,
        source_repo: &Path,
        bundle_path: &Path,
        request: &RepositoryVerificationRequest,
        policy_state: &RepositoryPolicyState,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<OfflineBundleManifest, OfflineBundleError> {
        let bundle_path = absolute_path(bundle_path)?;
        if bundle_path.exists() {
            return Err(OfflineBundleError::BundleAlreadyExists(bundle_path));
        }
        if let Some(parent) = bundle_path.parent() {
            fs::create_dir_all(parent)?;
        }

        // Capture must not silently hydrate source objects from a remote.
        self.ensure_complete_source(source_repo)?;

        let gittuf = self.gittuf_adapter();
        let invocation =
            GittufInvocation::from_request(request, policy_state, commitment_algorithm)?;
        let before = gittuf.observe(source_repo, &invocation)?;
        before.clone().into_observation_for(request)?;
        let before_commitment = before.commitment(DigestAlgorithm::Sha256)?;
        let expected_refs = expected_bundle_refs(request, &before)?;

        let mut args = vec![
            "bundle".to_owned(),
            "create".to_owned(),
            "--version=3".to_owned(),
            path_arg(&bundle_path)?,
        ];
        args.extend(
            expected_refs
                .iter()
                .map(|entry| entry.reference().to_string()),
        );
        let create = self.run_git(source_repo, args)?;
        require_success(&self.git_binary, "bundle create", &create)?;

        require_bundle_v3(&bundle_path)?;
        let advertised =
            self.bundle_heads(source_repo, &bundle_path, before.snapshot().object_format())?;
        require_exact_refs(&expected_refs, &advertised)?;
        let artifact = fingerprint_file(&bundle_path)?;

        // Race barrier around materialization.
        let after = gittuf.observe(source_repo, &invocation)?;
        after.clone().into_observation_for(request)?;
        let after_commitment = after.commitment(DigestAlgorithm::Sha256)?;
        if before_commitment != after_commitment {
            return Err(OfflineBundleError::SourceChangedDuringBundleCreation);
        }

        Ok(OfflineBundleManifest::new(
            request,
            policy_state.clone(),
            after_commitment,
            artifact.digest,
            artifact.size,
            after.snapshot().object_format(),
            advertised,
        )?)
    }

    /// Replay the exact repository closure into an empty repository and require
    /// the same FORGE-004B local verification subject.
    ///
    /// This proves portable repository closure, not a hermetic/network-free
    /// verifier environment. FORGE-004D is responsible for that stronger claim.
    pub fn replay_bundle(
        &self,
        bundle_path: &Path,
        replay_repo: &Path,
        manifest: &OfflineBundleManifest,
        request: &RepositoryVerificationRequest,
    ) -> Result<PortableRepositoryReplay, OfflineBundleError> {
        let bundle_path = absolute_path(bundle_path)?;
        let replay_repo = absolute_path(replay_repo)?;

        manifest.validate_for_request(request)?;
        require_bundle_v3(&bundle_path)?;
        let actual = fingerprint_file(&bundle_path)?;
        if &actual.digest != manifest.bundle_digest() || actual.size != manifest.bundle_size() {
            return Err(OfflineBundleError::BundleArtifactMismatch);
        }

        prepare_empty_directory(&replay_repo)?;
        let format = match manifest.object_format() {
            GitObjectAlgorithm::Sha1 => "sha1",
            GitObjectAlgorithm::Sha256 => "sha256",
        };
        let init = self.run_git(
            &replay_repo,
            vec![
                "init".to_owned(),
                "--bare".to_owned(),
                format!("--object-format={format}"),
                ".".to_owned(),
            ],
        )?;
        require_success(&self.git_binary, "init replay repository", &init)?;

        // Empty-repository verification rejects a bundle that relies on
        // prerequisite Git objects not carried in the artifact.
        let verify = self.run_git(
            &replay_repo,
            vec![
                "bundle".to_owned(),
                "verify".to_owned(),
                path_arg(&bundle_path)?,
            ],
        )?;
        require_success(&self.git_binary, "bundle verify", &verify)?;

        let advertised =
            self.bundle_heads(&replay_repo, &bundle_path, manifest.object_format())?;
        require_exact_refs(manifest.refs(), &advertised)?;

        let mut fetch_args = vec![
            "fetch".to_owned(),
            "--no-tags".to_owned(),
            "--no-write-fetch-head".to_owned(),
            path_arg(&bundle_path)?,
        ];
        for entry in manifest.refs() {
            let reference = entry.reference().as_str();
            fetch_args.push(format!("+{reference}:{reference}"));
        }
        let fetch = self.run_git(&replay_repo, fetch_args)?;
        require_success(&self.git_binary, "fetch repository closure", &fetch)?;

        let imported =
            self.read_exact_refs(&replay_repo, manifest.refs(), manifest.object_format())?;
        require_exact_refs(manifest.refs(), &imported)?;

        let invocation = GittufInvocation::from_request(
            request,
            manifest.policy_state(),
            manifest.policy_state_digest().algorithm(),
        )?;
        let replay_receipt = self.gittuf_adapter().observe(&replay_repo, &invocation)?;
        replay_receipt.clone().into_observation_for(request)?;
        let replay_commitment = replay_receipt.commitment(DigestAlgorithm::Sha256)?;
        if &replay_commitment != manifest.local_receipt_commitment() {
            return Err(OfflineBundleError::ReplayReceiptMismatch);
        }

        let replay_refs = expected_bundle_refs(request, &replay_receipt)?;
        require_exact_refs(manifest.refs(), &replay_refs)?;

        Ok(PortableRepositoryReplay {
            manifest: manifest.clone(),
            replay_receipt,
        })
    }

    fn gittuf_adapter(&self) -> GittufAdapter<R> {
        GittufAdapter::new(
            self.runner.clone(),
            self.gittuf_binary.clone(),
            self.git_binary.clone(),
        )
    }

    fn ensure_complete_source(&self, cwd: &Path) -> Result<(), OfflineBundleError> {
        let shallow = self.run_git(
            cwd,
            vec!["rev-parse".to_owned(), "--is-shallow-repository".to_owned()],
        )?;
        require_success(&self.git_binary, "check shallow repository", &shallow)?;
        if shallow.stdout.trim() != "false" {
            return Err(OfflineBundleError::ShallowSourceRepository);
        }

        let partial = self.run_git(
            cwd,
            vec![
                "config".to_owned(),
                "--local".to_owned(),
                "--get".to_owned(),
                "extensions.partialClone".to_owned(),
            ],
        )?;
        match partial.status {
            1 if partial.stdout.trim().is_empty() => {}
            0 => return Err(OfflineBundleError::PartialCloneSourceRepository),
            _ => {
                return Err(OfflineBundleError::CommandFailed {
                    program: self.git_binary.clone(),
                    operation: "check partial clone",
                    status: partial.status,
                    stderr: partial.stderr,
                })
            }
        }

        let promisor = self.run_git(
            cwd,
            vec![
                "config".to_owned(),
                "--local".to_owned(),
                "--get-regexp".to_owned(),
                r"^remote\..*\.promisor$".to_owned(),
            ],
        )?;
        match promisor.status {
            1 if promisor.stdout.trim().is_empty() => Ok(()),
            0 => Err(OfflineBundleError::PromisorRemoteConfigured),
            _ => Err(OfflineBundleError::CommandFailed {
                program: self.git_binary.clone(),
                operation: "check promisor remotes",
                status: promisor.status,
                stderr: promisor.stderr,
            }),
        }
    }

    fn bundle_heads(
        &self,
        cwd: &Path,
        bundle_path: &Path,
        object_format: GitObjectAlgorithm,
    ) -> Result<Vec<BundleRef>, OfflineBundleError> {
        let output = self.run_git(
            cwd,
            vec![
                "bundle".to_owned(),
                "list-heads".to_owned(),
                path_arg(bundle_path)?,
            ],
        )?;
        require_success(&self.git_binary, "bundle list-heads", &output)?;
        parse_bundle_heads(&output.stdout, object_format)
    }

    fn read_exact_refs(
        &self,
        cwd: &Path,
        refs: &[BundleRef],
        object_format: GitObjectAlgorithm,
    ) -> Result<Vec<BundleRef>, OfflineBundleError> {
        let mut observed = Vec::with_capacity(refs.len());
        for entry in refs {
            let output = self.run_git(
                cwd,
                vec![
                    "rev-parse".to_owned(),
                    "--verify".to_owned(),
                    entry.reference().to_string(),
                ],
            )?;
            require_success(&self.git_binary, "rev-parse imported ref", &output)?;
            observed.push(BundleRef::new(
                entry.reference().clone(),
                parse_git_object(output.stdout.trim(), object_format)?,
            ));
        }
        Ok(canonicalize_refs(observed)?)
    }

    fn run_git(
        &self,
        cwd: &Path,
        args: Vec<String>,
    ) -> Result<CommandOutput, OfflineBundleError> {
        Ok(self.runner.run(&CommandSpec {
            program: self.git_binary.clone(),
            args,
            cwd: cwd.to_path_buf(),
            env: vec![
                ("LC_ALL".to_owned(), "C".to_owned()),
                ("LANG".to_owned(), "C".to_owned()),
                ("GITTUF_DEV".to_owned(), "0".to_owned()),
                ("GITTUF_DEBUG".to_owned(), "0".to_owned()),
                ("GIT_NO_LAZY_FETCH".to_owned(), "1".to_owned()),
                ("GIT_TERMINAL_PROMPT".to_owned(), "0".to_owned()),
                ("GIT_CONFIG_NOSYSTEM".to_owned(), "1".to_owned()),
            ],
        })?)
    }
}

pub fn expected_bundle_refs(
    request: &RepositoryVerificationRequest,
    receipt: &GittufLocalReceipt,
) -> Result<Vec<BundleRef>, OfflineBundleError> {
    receipt.clone().into_observation_for(request)?;
    let snapshot = receipt.snapshot();
    let mut refs = vec![
        BundleRef::new(request.reference().clone(), request.to().clone()),
        BundleRef::new(RepositoryRef::new(RSL_REF)?, snapshot.rsl_tip().clone()),
        BundleRef::new(
            RepositoryRef::new(POLICY_REF)?,
            snapshot.policy_tip().clone(),
        ),
    ];
    if let Some(attestations) = snapshot.attestations_tip() {
        refs.push(BundleRef::new(
            RepositoryRef::new(ATTESTATIONS_REF)?,
            attestations.clone(),
        ));
    }
    Ok(canonicalize_refs(refs)?)
}

pub fn fingerprint_file(path: &Path) -> Result<ArtifactFingerprint, OfflineBundleError> {
    let mut file = File::open(path)?;
    let mut hasher = Sha256::new();
    let mut size = 0_u64;
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let read = file.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
        size = size
            .checked_add(read as u64)
            .ok_or(OfflineBundleError::ArtifactTooLarge)?;
    }
    let digest = Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?;
    Ok(ArtifactFingerprint { digest, size })
}

pub fn require_bundle_v3(path: &Path) -> Result<(), OfflineBundleError> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut first = String::new();
    reader.read_line(&mut first)?;
    if first.trim_end_matches(&['\r', '\n'][..]) != BUNDLE_V3_HEADER {
        return Err(OfflineBundleError::NotBundleV3);
    }
    Ok(())
}

pub fn parse_bundle_heads(
    stdout: &str,
    object_format: GitObjectAlgorithm,
) -> Result<Vec<BundleRef>, OfflineBundleError> {
    let mut refs = Vec::new();
    for line in stdout.lines() {
        let mut fields = line.split_whitespace();
        let object = fields
            .next()
            .ok_or_else(|| OfflineBundleError::MalformedBundleHead(line.to_owned()))?;
        let reference = fields
            .next()
            .ok_or_else(|| OfflineBundleError::MalformedBundleHead(line.to_owned()))?;
        if fields.next().is_some() {
            return Err(OfflineBundleError::MalformedBundleHead(line.to_owned()));
        }
        refs.push(BundleRef::new(
            RepositoryRef::new(reference)?,
            parse_git_object(object, object_format)?,
        ));
    }
    Ok(canonicalize_refs(refs)?)
}

fn parse_git_object(
    value: &str,
    object_format: GitObjectAlgorithm,
) -> Result<GitObjectId, OfflineBundleError> {
    let expected = object_format.digest_len() * 2;
    if value.len() != expected || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(OfflineBundleError::InvalidGitObject(value.to_owned()));
    }
    let mut bytes = Vec::with_capacity(object_format.digest_len());
    for pair in value.as_bytes().chunks_exact(2) {
        let high = hex_nibble(pair[0])?;
        let low = hex_nibble(pair[1])?;
        bytes.push((high << 4) | low);
    }
    Ok(GitObjectId::new(object_format, bytes)?)
}

fn hex_nibble(byte: u8) -> Result<u8, OfflineBundleError> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err(OfflineBundleError::InvalidHexDigit(byte)),
    }
}

fn require_exact_refs(expected: &[BundleRef], actual: &[BundleRef]) -> Result<(), OfflineBundleError> {
    if expected == actual {
        Ok(())
    } else {
        Err(OfflineBundleError::BundleRefSetMismatch)
    }
}

fn prepare_empty_directory(path: &Path) -> Result<(), OfflineBundleError> {
    if path.exists() {
        let mut entries = fs::read_dir(path)?;
        if entries.next().transpose()?.is_some() {
            return Err(OfflineBundleError::ReplayDirectoryNotEmpty(
                path.to_path_buf(),
            ));
        }
    } else {
        fs::create_dir_all(path)?;
    }
    Ok(())
}

fn absolute_path(path: &Path) -> Result<PathBuf, OfflineBundleError> {
    if path.is_absolute() {
        Ok(path.to_path_buf())
    } else {
        Ok(std::env::current_dir()?.join(path))
    }
}

fn path_arg(path: &Path) -> Result<String, OfflineBundleError> {
    path.to_str()
        .map(ToOwned::to_owned)
        .ok_or_else(|| OfflineBundleError::NonUtf8Path(path.to_path_buf()))
}

fn require_success(
    program: &Path,
    operation: &'static str,
    output: &CommandOutput,
) -> Result<(), OfflineBundleError> {
    if output.status == 0 {
        Ok(())
    } else {
        Err(OfflineBundleError::CommandFailed {
            program: program.to_path_buf(),
            operation,
            status: output.status,
            stderr: output.stderr.clone(),
        })
    }
}

#[derive(Debug, Error)]
pub enum OfflineBundleError {
    #[error(transparent)]
    Core(#[from] ForgeCoreError),
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error(transparent)]
    Gittuf(#[from] AdapterError),
    #[error(transparent)]
    Manifest(#[from] OfflineManifestError),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("bundle path already exists: {0:?}")]
    BundleAlreadyExists(PathBuf),
    #[error("source repository is shallow; hydrate full history before capture")]
    ShallowSourceRepository,
    #[error("source repository is a partial clone; hydrate it before capture")]
    PartialCloneSourceRepository,
    #[error("source repository has a promisor remote; hydrate and remove promisor dependence before capture")]
    PromisorRemoteConfigured,
    #[error("bundle is not Git bundle v3")]
    NotBundleV3,
    #[error("bundle artifact digest or size does not match manifest")]
    BundleArtifactMismatch,
    #[error("source repository changed while materializing repository closure")]
    SourceChangedDuringBundleCreation,
    #[error("replayed local verification receipt does not match source receipt")]
    ReplayReceiptMismatch,
    #[error("bundle advertised refs do not exactly match expected refs")]
    BundleRefSetMismatch,
    #[error("malformed git bundle list-heads line: {0}")]
    MalformedBundleHead(String),
    #[error("invalid Git object ID: {0}")]
    InvalidGitObject(String),
    #[error("invalid hexadecimal digit byte {0}")]
    InvalidHexDigit(u8),
    #[error("bundle artifact size overflow")]
    ArtifactTooLarge,
    #[error("replay directory must be empty: {0:?}")]
    ReplayDirectoryNotEmpty(PathBuf),
    #[error("path is not valid UTF-8: {0:?}")]
    NonUtf8Path(PathBuf),
    #[error("command failed during {operation}: {program:?}, status={status}, stderr={stderr}")]
    CommandFailed {
        program: PathBuf,
        operation: &'static str,
        status: i32,
        stderr: String,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::tempdir;

    fn git_sha1(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn hex_id(byte: u8) -> String {
        format!("{byte:02x}").repeat(20)
    }

    #[test]
    fn bundle_head_parser_canonicalizes_order() {
        let stdout = format!(
            "{} refs/gittuf/policy\n{} refs/heads/main\n",
            hex_id(0x99),
            hex_id(0x77)
        );
        let refs = parse_bundle_heads(&stdout, GitObjectAlgorithm::Sha1).unwrap();
        assert_eq!(refs[0].reference().as_str(), "refs/gittuf/policy");
        assert_eq!(refs[1].reference().as_str(), "refs/heads/main");
        assert_eq!(refs[1].tip(), &git_sha1(0x77));
    }

    #[test]
    fn malformed_bundle_head_is_rejected() {
        assert!(matches!(
            parse_bundle_heads("deadbeef refs/heads/main extra\n", GitObjectAlgorithm::Sha1),
            Err(OfflineBundleError::MalformedBundleHead(_))
        ));
    }

    #[test]
    fn artifact_fingerprint_detects_byte_mutation() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("evidence.bundle");
        fs::write(&path, b"alpha").unwrap();
        let first = fingerprint_file(&path).unwrap();
        fs::write(&path, b"alphb").unwrap();
        let second = fingerprint_file(&path).unwrap();
        assert_eq!(first.size, second.size);
        assert_ne!(first.digest, second.digest);
    }

    #[test]
    fn v3_header_is_required() {
        let dir = tempdir().unwrap();
        let good = dir.path().join("good.bundle");
        let bad = dir.path().join("bad.bundle");
        let mut file = File::create(&good).unwrap();
        writeln!(file, "# v3 git bundle").unwrap();
        fs::write(&bad, b"# v2 git bundle\n").unwrap();
        require_bundle_v3(&good).unwrap();
        assert!(matches!(
            require_bundle_v3(&bad),
            Err(OfflineBundleError::NotBundleV3)
        ));
    }

    #[test]
    fn replay_directory_must_be_empty() {
        let dir = tempdir().unwrap();
        fs::write(dir.path().join("unexpected"), b"x").unwrap();
        assert!(matches!(
            prepare_empty_directory(dir.path()),
            Err(OfflineBundleError::ReplayDirectoryNotEmpty(_))
        ));
    }
}

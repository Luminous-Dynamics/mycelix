// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2D1: sealed input transport for Forge hermetic execution.
//!
//! A path-based read-only bind still leaves a host inode mutable outside the
//! sandbox. M0 therefore snapshots every non-Nix input into a sealed anonymous
//! file, re-hashes the sealed bytes, and asks bubblewrap to bind the exact FD
//! read-only with `--ro-bind-fd`.

use mycelix_forge_core::{Digest, DigestAlgorithm, ForgeCoreError};
use mycelix_forge_linux_isolation::{
    build_bubblewrap_command, BubblewrapCommand, IsolationPolicyError, LinuxIsolationPolicyV1,
    NixClosureManifest, RuntimeArtifact, VerifierInvocation,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest as ShaDigest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    ffi::CString,
    fs::File,
    io::{Read, Seek, SeekFrom, Write},
    os::fd::{AsRawFd, FromRawFd, RawFd},
    path::{Path, PathBuf},
};
use thiserror::Error;

const SEALED_INPUT_DOMAIN_V1: &[u8] = b"mycelix-forge/sealed-input-evidence/v1\0";
const REQUIRED_SEALS: i32 =
    libc::F_SEAL_WRITE | libc::F_SEAL_GROW | libc::F_SEAL_SHRINK | libc::F_SEAL_SEAL;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SealedArtifactObservation {
    role: String,
    destination: String,
    digest: Digest,
    size: u64,
    seals: i32,
}

impl SealedArtifactObservation {
    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn destination(&self) -> &str {
        &self.destination
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }

    pub const fn seals(&self) -> i32 {
        self.seals
    }

    pub const fn fully_sealed(&self) -> bool {
        self.seals & REQUIRED_SEALS == REQUIRED_SEALS
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SealedInputObservation {
    policy_digest: Digest,
    artifacts: Vec<SealedArtifactObservation>,
}

impl SealedInputObservation {
    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn artifacts(&self) -> &[SealedArtifactObservation] {
        &self.artifacts
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, SealedInputError> {
        validate_observation_order(&self.artifacts)?;
        let mut out = Vec::new();
        out.extend_from_slice(SEALED_INPUT_DOMAIN_V1);
        push_digest(&mut out, &self.policy_digest)?;
        push_count(&mut out, self.artifacts.len(), "sealed artifacts")?;
        for artifact in &self.artifacts {
            push_string(&mut out, artifact.role(), "role")?;
            push_string(&mut out, artifact.destination(), "destination")?;
            push_digest(&mut out, artifact.digest())?;
            out.extend_from_slice(&artifact.size().to_be_bytes());
            out.extend_from_slice(&artifact.seals().to_be_bytes());
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, SealedInputError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedSealedInputEvidence {
    policy_digest: Digest,
    evidence_digest: Digest,
    artifacts: Vec<SealedArtifactObservation>,
}

impl QualifiedSealedInputEvidence {
    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }

    pub fn artifacts(&self) -> &[SealedArtifactObservation] {
        &self.artifacts
    }
}

pub struct SealedRuntimeArtifact {
    role: String,
    destination: String,
    file: File,
    observation: SealedArtifactObservation,
}

impl SealedRuntimeArtifact {
    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn destination(&self) -> &str {
        &self.destination
    }

    pub fn raw_fd(&self) -> RawFd {
        self.file.as_raw_fd()
    }

    pub fn observation(&self) -> &SealedArtifactObservation {
        &self.observation
    }

    fn rewind(&mut self) -> Result<(), SealedInputError> {
        self.file.seek(SeekFrom::Start(0))?;
        Ok(())
    }
}

pub struct SealedRuntimeInputs {
    policy_digest: Digest,
    artifacts: Vec<SealedRuntimeArtifact>,
    qualified: QualifiedSealedInputEvidence,
}

impl SealedRuntimeInputs {
    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn qualified(&self) -> &QualifiedSealedInputEvidence {
        &self.qualified
    }

    pub fn artifacts(&self) -> &[SealedRuntimeArtifact] {
        &self.artifacts
    }

    pub fn artifact(&self, role: &str) -> Option<&SealedRuntimeArtifact> {
        self.artifacts.iter().find(|artifact| artifact.role() == role)
    }

    fn rewind_all(&mut self) -> Result<(), SealedInputError> {
        for artifact in &mut self.artifacts {
            artifact.rewind()?;
        }
        Ok(())
    }
}

/// Snapshot every runtime artifact into an immutable memfd.
pub fn seal_runtime_inputs(
    policy: &LinuxIsolationPolicyV1,
    runtime_artifacts: &[RuntimeArtifact],
) -> Result<SealedRuntimeInputs, SealedInputError> {
    let runtime = runtime_artifacts
        .iter()
        .map(|artifact| (artifact.role.as_str(), artifact.source.as_path()))
        .collect::<BTreeMap<_, _>>();
    if runtime.len() != runtime_artifacts.len() {
        return Err(SealedInputError::DuplicateRuntimeRole);
    }

    let expected_roles = policy
        .artifact_mounts()
        .iter()
        .map(|mount| mount.role())
        .collect::<BTreeSet<_>>();
    let actual_roles = runtime.keys().copied().collect::<BTreeSet<_>>();
    if actual_roles != expected_roles {
        return Err(SealedInputError::RuntimeRoleSetMismatch);
    }

    let mut artifacts = Vec::with_capacity(policy.artifact_mounts().len());
    for mount in policy.artifact_mounts() {
        let source = runtime
            .get(mount.role())
            .expect("exact runtime role set checked");
        artifacts.push(seal_one(
            mount.role(),
            mount.destination(),
            mount.digest(),
            mount.size(),
            source,
        )?);
    }
    artifacts.sort_by(|a, b| a.role.cmp(&b.role));

    let policy_digest = policy.digest(DigestAlgorithm::Sha256)?;
    let observation = SealedInputObservation {
        policy_digest: policy_digest.clone(),
        artifacts: artifacts
            .iter()
            .map(|artifact| artifact.observation.clone())
            .collect(),
    };
    let qualified = qualify_sealed_inputs(policy, &observation)?;
    Ok(SealedRuntimeInputs {
        policy_digest,
        artifacts,
        qualified,
    })
}

pub fn qualify_sealed_inputs(
    policy: &LinuxIsolationPolicyV1,
    observation: &SealedInputObservation,
) -> Result<QualifiedSealedInputEvidence, SealedInputError> {
    let policy_digest = policy.digest(DigestAlgorithm::Sha256)?;
    if observation.policy_digest != policy_digest {
        return Err(SealedInputError::PolicyDigestMismatch);
    }
    validate_observation_order(&observation.artifacts)?;

    if observation.artifacts.len() != policy.artifact_mounts().len() {
        return Err(SealedInputError::ArtifactSetMismatch);
    }
    for (observed, expected) in observation.artifacts.iter().zip(policy.artifact_mounts()) {
        if observed.role() != expected.role()
            || observed.destination() != expected.destination()
            || observed.digest() != expected.digest()
            || observed.size() != expected.size()
        {
            return Err(SealedInputError::ArtifactMismatch(expected.role().to_owned()));
        }
        if !observed.fully_sealed() {
            return Err(SealedInputError::MissingRequiredSeals(
                expected.role().to_owned(),
            ));
        }
    }

    Ok(QualifiedSealedInputEvidence {
        policy_digest,
        evidence_digest: observation.digest(DigestAlgorithm::Sha256)?,
        artifacts: observation.artifacts.clone(),
    })
}

/// Build the canonical bubblewrap command, replacing only policy input mounts
/// with `--ro-bind-fd FD DEST`. Nix closure mounts remain path-based and are
/// separately protected by NAR qualification.
pub fn build_sealed_bubblewrap_command(
    bubblewrap_program: impl Into<PathBuf>,
    policy: &LinuxIsolationPolicyV1,
    closure: &NixClosureManifest,
    invocation: &VerifierInvocation,
    sealed: &mut SealedRuntimeInputs,
) -> Result<BubblewrapCommand, SealedInputError> {
    let policy_digest = policy.digest(DigestAlgorithm::Sha256)?;
    if sealed.policy_digest != policy_digest || sealed.qualified.policy_digest() != &policy_digest {
        return Err(SealedInputError::PolicyDigestMismatch);
    }
    sealed.rewind_all()?;

    let synthetic = sealed
        .artifacts
        .iter()
        .map(|artifact| RuntimeArtifact {
            role: artifact.role.clone(),
            source: PathBuf::from(format!("/proc/self/fd/{}", artifact.raw_fd())),
        })
        .collect::<Vec<_>>();
    let mut command = build_bubblewrap_command(
        bubblewrap_program,
        policy,
        closure,
        invocation,
        &synthetic,
    )?;

    let fd_by_destination = sealed
        .artifacts
        .iter()
        .map(|artifact| (artifact.destination.as_str(), artifact.raw_fd()))
        .collect::<BTreeMap<_, _>>();
    let mut rewritten = BTreeSet::new();
    let mut index = 0;
    while index + 2 < command.args.len() {
        if command.args[index] == "--ro-bind" {
            let destination = command.args[index + 2].as_str();
            if let Some(fd) = fd_by_destination.get(destination) {
                command.args[index] = "--ro-bind-fd".to_owned();
                command.args[index + 1] = fd.to_string();
                rewritten.insert(destination.to_owned());
            }
            index += 3;
        } else {
            index += 1;
        }
    }

    if rewritten.len() != policy.artifact_mounts().len() {
        return Err(SealedInputError::BubblewrapRewriteMismatch);
    }
    Ok(command)
}

fn seal_one(
    role: &str,
    destination: &str,
    expected_digest: &Digest,
    expected_size: u64,
    source: &Path,
) -> Result<SealedRuntimeArtifact, SealedInputError> {
    let name = CString::new(format!("mycelix-forge-{role}"))
        .map_err(|_| SealedInputError::InvalidMemfdName)?;
    let fd = unsafe { libc::memfd_create(name.as_ptr(), libc::MFD_ALLOW_SEALING as u32) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    let mut memfd = unsafe { File::from_raw_fd(fd) };
    let mut source_file = File::open(source)?;
    let (digest, size) = copy_with_digest(&mut source_file, &mut memfd, expected_digest.algorithm())?;
    if &digest != expected_digest || size != expected_size {
        return Err(SealedInputError::SourceArtifactMismatch(role.to_owned()));
    }
    memfd.flush()?;

    let result = unsafe { libc::fcntl(memfd.as_raw_fd(), libc::F_ADD_SEALS, REQUIRED_SEALS) };
    if result != 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    let seals = unsafe { libc::fcntl(memfd.as_raw_fd(), libc::F_GET_SEALS) };
    if seals < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    if seals & REQUIRED_SEALS != REQUIRED_SEALS {
        return Err(SealedInputError::MissingRequiredSeals(role.to_owned()));
    }

    memfd.seek(SeekFrom::Start(0))?;
    let (sealed_digest, sealed_size) = digest_reader(&mut memfd, expected_digest.algorithm())?;
    if sealed_digest != *expected_digest || sealed_size != expected_size {
        return Err(SealedInputError::SealedRereadMismatch(role.to_owned()));
    }
    memfd.seek(SeekFrom::Start(0))?;

    let observation = SealedArtifactObservation {
        role: role.to_owned(),
        destination: destination.to_owned(),
        digest: sealed_digest,
        size: sealed_size,
        seals,
    };
    Ok(SealedRuntimeArtifact {
        role: role.to_owned(),
        destination: destination.to_owned(),
        file: memfd,
        observation,
    })
}

fn copy_with_digest(
    source: &mut File,
    destination: &mut File,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), SealedInputError> {
    let mut buffer = [0_u8; 64 * 1024];
    let mut size = 0_u64;
    match algorithm {
        DigestAlgorithm::Sha256 => {
            let mut hasher = Sha256::new();
            loop {
                let read = source.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                destination.write_all(&buffer[..read])?;
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(SealedInputError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?,
                size,
            ))
        }
        DigestAlgorithm::Blake3_256 => {
            let mut hasher = blake3::Hasher::new();
            loop {
                let read = source.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                destination.write_all(&buffer[..read])?;
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(SealedInputError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(
                    DigestAlgorithm::Blake3_256,
                    hasher.finalize().as_bytes().to_vec(),
                )?,
                size,
            ))
        }
    }
}

fn digest_reader(
    reader: &mut File,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), SealedInputError> {
    let mut buffer = [0_u8; 64 * 1024];
    let mut size = 0_u64;
    match algorithm {
        DigestAlgorithm::Sha256 => {
            let mut hasher = Sha256::new();
            loop {
                let read = reader.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(SealedInputError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?,
                size,
            ))
        }
        DigestAlgorithm::Blake3_256 => {
            let mut hasher = blake3::Hasher::new();
            loop {
                let read = reader.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(SealedInputError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(
                    DigestAlgorithm::Blake3_256,
                    hasher.finalize().as_bytes().to_vec(),
                )?,
                size,
            ))
        }
    }
}

fn validate_observation_order(
    artifacts: &[SealedArtifactObservation],
) -> Result<(), SealedInputError> {
    let mut sorted = artifacts.to_vec();
    sorted.sort();
    if sorted != artifacts {
        return Err(SealedInputError::ArtifactsNotCanonical);
    }
    let mut roles = BTreeSet::new();
    let mut destinations = BTreeSet::new();
    for artifact in artifacts {
        if !roles.insert(artifact.role()) || !destinations.insert(artifact.destination()) {
            return Err(SealedInputError::DuplicateArtifact);
        }
    }
    Ok(())
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), SealedInputError> {
    let count =
        u16::try_from(count).map_err(|_| SealedInputError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), SealedInputError> {
    let len =
        u16::try_from(value.len()).map_err(|_| SealedInputError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), SealedInputError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| SealedInputError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum SealedInputError {
    #[error(transparent)]
    Core(#[from] ForgeCoreError),
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("duplicate runtime artifact role")]
    DuplicateRuntimeRole,
    #[error("runtime artifact role set differs from isolation policy")]
    RuntimeRoleSetMismatch,
    #[error("sealed-input observation names a different policy")]
    PolicyDigestMismatch,
    #[error("sealed artifact set differs from policy")]
    ArtifactSetMismatch,
    #[error("sealed artifact differs from policy: {0}")]
    ArtifactMismatch(String),
    #[error("artifact is missing required immutable seals: {0}")]
    MissingRequiredSeals(String),
    #[error("source artifact bytes differ from policy: {0}")]
    SourceArtifactMismatch(String),
    #[error("sealed artifact re-read differs from policy: {0}")]
    SealedRereadMismatch(String),
    #[error("sealed artifact observations are not canonical")]
    ArtifactsNotCanonical,
    #[error("duplicate sealed artifact role or destination")]
    DuplicateArtifact,
    #[error("bubblewrap artifact transport rewrite was incomplete")]
    BubblewrapRewriteMismatch,
    #[error("invalid memfd name")]
    InvalidMemfdName,
    #[error("artifact byte length overflow")]
    ArtifactTooLarge,
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

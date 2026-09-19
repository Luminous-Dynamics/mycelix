// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2D2D: bind declared tool artifacts to executed file bytes.
//!
//! The Nix closure authenticates the filesystem trees containing executables.
//! `ExecutionSpec::ToolArtifact` separately commits tool-role metadata. This
//! crate closes the remaining relation between those two facts by hashing the
//! actual executable file selected for every declared tool role.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_execution::{ExecutionContractError, ExecutionSpec, ToolArtifact};
use mycelix_forge_linux_isolation::{IsolationPolicyError, NixClosureManifest};
use sha2::{Digest as ShaDigest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::{self, File},
    io::Read,
    os::unix::fs::PermissionsExt,
    path::{Component, Path},
};
use thiserror::Error;

const TOOL_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/runtime-tool-evidence/v1\0";
const MAX_TEXT: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct RuntimeToolObservation {
    role: String,
    executable: String,
    resolved_executable: String,
    named_store_root: String,
    resolved_store_root: String,
    digest: Digest,
    size: u64,
}

impl RuntimeToolObservation {
    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn executable(&self) -> &str {
        &self.executable
    }

    pub fn resolved_executable(&self) -> &str {
        &self.resolved_executable
    }

    pub fn named_store_root(&self) -> &str {
        &self.named_store_root
    }

    pub fn resolved_store_root(&self) -> &str {
        &self.resolved_store_root
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedRuntimeToolEvidence {
    execution_spec_digest: Digest,
    closure_digest: Digest,
    observations: Vec<RuntimeToolObservation>,
    evidence_digest: Digest,
}

impl QualifiedRuntimeToolEvidence {
    pub fn execution_spec_digest(&self) -> &Digest {
        &self.execution_spec_digest
    }

    pub fn closure_digest(&self) -> &Digest {
        &self.closure_digest
    }

    pub fn observations(&self) -> &[RuntimeToolObservation] {
        &self.observations
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }

    pub fn executable(&self, role: &str) -> Option<&str> {
        self.observations
            .iter()
            .find(|observation| observation.role() == role)
            .map(RuntimeToolObservation::executable)
    }
}

/// Hash the actual executable bytes selected for every tool role in `spec`.
///
/// `paths` must name exactly the complete tool-role set from the execution
/// specification. Both the named path and its symlink-resolved target must live
/// under store roots committed by `closure`.
pub fn audit_runtime_tools(
    spec: &ExecutionSpec,
    closure: &NixClosureManifest,
    paths: &BTreeMap<String, String>,
) -> Result<QualifiedRuntimeToolEvidence, RuntimeToolEvidenceError> {
    let expected_roles = spec
        .tools()
        .iter()
        .map(ToolArtifact::role)
        .collect::<BTreeSet<_>>();
    let actual_roles = paths.keys().map(String::as_str).collect::<BTreeSet<_>>();
    if actual_roles != expected_roles {
        return Err(RuntimeToolEvidenceError::ToolRoleSetMismatch);
    }

    let mut observations = Vec::with_capacity(spec.tools().len());
    let mut named_paths = BTreeSet::new();
    let mut resolved_paths = BTreeSet::new();

    for tool in spec.tools() {
        let executable = paths
            .get(tool.role())
            .expect("exact tool role set checked");
        let observation = audit_one(tool, executable, closure)?;
        if !named_paths.insert(observation.executable.clone()) {
            return Err(RuntimeToolEvidenceError::DuplicateExecutablePath(
                observation.executable,
            ));
        }
        if !resolved_paths.insert(observation.resolved_executable.clone()) {
            return Err(RuntimeToolEvidenceError::DuplicateResolvedExecutablePath(
                observation.resolved_executable,
            ));
        }
        observations.push(observation);
    }
    observations.sort();

    let execution_spec_digest = spec.digest(DigestAlgorithm::Sha256)?;
    let closure_digest = closure.digest(DigestAlgorithm::Sha256)?;
    let evidence_digest = derive_evidence_digest(
        &execution_spec_digest,
        &closure_digest,
        &observations,
    )?;

    Ok(QualifiedRuntimeToolEvidence {
        execution_spec_digest,
        closure_digest,
        observations,
        evidence_digest,
    })
}

fn audit_one(
    tool: &ToolArtifact,
    executable: &str,
    closure: &NixClosureManifest,
) -> Result<RuntimeToolObservation, RuntimeToolEvidenceError> {
    let named_store_root = require_committed_store_path(executable, closure)?;
    let resolved = fs::canonicalize(executable)?;
    let resolved_executable = resolved
        .to_str()
        .ok_or(RuntimeToolEvidenceError::NonUtf8ResolvedExecutable)?
        .to_owned();
    let resolved_store_root = require_committed_store_path(&resolved_executable, closure)?;

    let metadata = fs::metadata(&resolved)?;
    if !metadata.is_file() {
        return Err(RuntimeToolEvidenceError::ExecutableNotRegularFile(
            tool.role().to_owned(),
        ));
    }
    if metadata.permissions().mode() & 0o111 == 0 {
        return Err(RuntimeToolEvidenceError::ExecutableBitMissing(
            tool.role().to_owned(),
        ));
    }

    let (digest, size) = hash_file(&resolved, tool.digest().algorithm())?;
    if &digest != tool.digest() {
        return Err(RuntimeToolEvidenceError::ToolDigestMismatch(
            tool.role().to_owned(),
        ));
    }
    if size != tool.size() {
        return Err(RuntimeToolEvidenceError::ToolSizeMismatch {
            role: tool.role().to_owned(),
            expected: tool.size(),
            actual: size,
        });
    }

    Ok(RuntimeToolObservation {
        role: tool.role().to_owned(),
        executable: executable.to_owned(),
        resolved_executable,
        named_store_root,
        resolved_store_root,
        digest,
        size,
    })
}

fn require_committed_store_path(
    executable: &str,
    closure: &NixClosureManifest,
) -> Result<String, RuntimeToolEvidenceError> {
    let root = store_root_from_path(executable)?;
    if !closure
        .entries()
        .iter()
        .any(|entry| entry.store_path() == root)
    {
        return Err(RuntimeToolEvidenceError::ExecutableOutsideClosure(
            executable.to_owned(),
        ));
    }
    Ok(root)
}

fn store_root_from_path(value: &str) -> Result<String, RuntimeToolEvidenceError> {
    if value.is_empty()
        || value.len() > MAX_TEXT
        || value.contains('\0')
        || !value.starts_with("/nix/store/")
        || value.ends_with('/')
    {
        return Err(RuntimeToolEvidenceError::InvalidExecutablePath(
            value.to_owned(),
        ));
    }
    for component in Path::new(value).components() {
        if matches!(component, Component::CurDir | Component::ParentDir) {
            return Err(RuntimeToolEvidenceError::InvalidExecutablePath(
                value.to_owned(),
            ));
        }
    }

    let tail = value
        .strip_prefix("/nix/store/")
        .ok_or_else(|| RuntimeToolEvidenceError::InvalidExecutablePath(value.to_owned()))?;
    let (entry, remainder) = tail
        .split_once('/')
        .ok_or_else(|| RuntimeToolEvidenceError::InvalidExecutablePath(value.to_owned()))?;
    if entry.is_empty() || remainder.is_empty() {
        return Err(RuntimeToolEvidenceError::InvalidExecutablePath(
            value.to_owned(),
        ));
    }
    Ok(format!("/nix/store/{entry}"))
}

fn hash_file(
    path: &Path,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), RuntimeToolEvidenceError> {
    let mut file = File::open(path)?;
    let mut buffer = [0_u8; 64 * 1024];
    let mut size = 0_u64;

    match algorithm {
        DigestAlgorithm::Sha256 => {
            let mut hasher = Sha256::new();
            loop {
                let read = file.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                let read = u64::try_from(read)
                    .map_err(|_| RuntimeToolEvidenceError::ArtifactTooLarge)?;
                size = size
                    .checked_add(read)
                    .ok_or(RuntimeToolEvidenceError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?,
                size,
            ))
        }
        DigestAlgorithm::Blake3_256 => {
            let mut hasher = blake3::Hasher::new();
            loop {
                let read = file.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                let read = u64::try_from(read)
                    .map_err(|_| RuntimeToolEvidenceError::ArtifactTooLarge)?;
                size = size
                    .checked_add(read)
                    .ok_or(RuntimeToolEvidenceError::ArtifactTooLarge)?;
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

fn derive_evidence_digest(
    execution_spec: &Digest,
    closure: &Digest,
    observations: &[RuntimeToolObservation],
) -> Result<Digest, RuntimeToolEvidenceError> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(TOOL_EVIDENCE_DOMAIN_V1);
    push_digest(&mut bytes, execution_spec)?;
    push_digest(&mut bytes, closure)?;
    let count = u16::try_from(observations.len())
        .map_err(|_| RuntimeToolEvidenceError::CanonicalLengthOverflow("tools"))?;
    bytes.extend_from_slice(&count.to_be_bytes());
    for observation in observations {
        push_string(&mut bytes, observation.role(), "tool role")?;
        push_string(&mut bytes, observation.executable(), "tool executable")?;
        push_string(
            &mut bytes,
            observation.resolved_executable(),
            "resolved executable",
        )?;
        push_string(
            &mut bytes,
            observation.named_store_root(),
            "named store root",
        )?;
        push_string(
            &mut bytes,
            observation.resolved_store_root(),
            "resolved store root",
        )?;
        push_digest(&mut bytes, observation.digest())?;
        bytes.extend_from_slice(&observation.size().to_be_bytes());
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), RuntimeToolEvidenceError> {
    let len = u16::try_from(value.len())
        .map_err(|_| RuntimeToolEvidenceError::CanonicalLengthOverflow(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), RuntimeToolEvidenceError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| RuntimeToolEvidenceError::CanonicalLengthOverflow("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum RuntimeToolEvidenceError {
    #[error(transparent)]
    Core(#[from] mycelix_forge_core::ForgeCoreError),
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("runtime executable path-role set differs from ExecutionSpec tool roles")]
    ToolRoleSetMismatch,
    #[error("invalid runtime executable path: {0}")]
    InvalidExecutablePath(String),
    #[error("runtime executable is outside the committed Nix closure: {0}")]
    ExecutableOutsideClosure(String),
    #[error("resolved runtime executable path is non-UTF-8")]
    NonUtf8ResolvedExecutable,
    #[error("two tool roles name the same executable path: {0}")]
    DuplicateExecutablePath(String),
    #[error("two tool roles resolve to the same executable file: {0}")]
    DuplicateResolvedExecutablePath(String),
    #[error("runtime tool is not a regular file: {0}")]
    ExecutableNotRegularFile(String),
    #[error("runtime tool has no executable permission bits: {0}")]
    ExecutableBitMissing(String),
    #[error("runtime tool digest differs from ExecutionSpec: {0}")]
    ToolDigestMismatch(String),
    #[error("runtime tool size differs from ExecutionSpec for {role}: expected {expected}, got {actual}")]
    ToolSizeMismatch {
        role: String,
        expected: u64,
        actual: u64,
    },
    #[error("runtime tool byte length overflow")]
    ArtifactTooLarge,
    #[error("canonical field length overflow: {0}")]
    CanonicalLengthOverflow(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{fs, os::unix::fs::PermissionsExt};
    use tempfile::tempdir;

    #[test]
    fn store_root_parser_rejects_lexical_escape() {
        assert!(matches!(
            store_root_from_path("/nix/store/aaaa-test/../bin/tool"),
            Err(RuntimeToolEvidenceError::InvalidExecutablePath(_))
        ));
        assert!(matches!(
            store_root_from_path("/usr/bin/git"),
            Err(RuntimeToolEvidenceError::InvalidExecutablePath(_))
        ));
    }

    #[test]
    fn file_hashing_tracks_exact_bytes_and_size() {
        let temp = tempdir().unwrap();
        let path = temp.path().join("tool");
        fs::write(&path, b"#!/bin/sh\necho forge\n").unwrap();
        let mut permissions = fs::metadata(&path).unwrap().permissions();
        permissions.set_mode(0o755);
        fs::set_permissions(&path, permissions).unwrap();

        let (sha, size) = hash_file(&path, DigestAlgorithm::Sha256).unwrap();
        assert_eq!(size, 21);
        assert_eq!(sha.as_bytes().len(), 32);

        let (blake, blake_size) = hash_file(&path, DigestAlgorithm::Blake3_256).unwrap();
        assert_eq!(blake_size, size);
        assert_ne!(sha, blake);
    }

    #[test]
    fn evidence_digest_binds_selected_path() {
        let digest = Digest::new(DigestAlgorithm::Sha256, vec![1; 32]).unwrap();
        let observation = |path: &str| RuntimeToolObservation {
            role: "git".to_owned(),
            executable: path.to_owned(),
            resolved_executable: path.to_owned(),
            named_store_root: "/nix/store/aaaa-git".to_owned(),
            resolved_store_root: "/nix/store/aaaa-git".to_owned(),
            digest: digest.clone(),
            size: 42,
        };
        let spec = Digest::new(DigestAlgorithm::Sha256, vec![2; 32]).unwrap();
        let closure = Digest::new(DigestAlgorithm::Sha256, vec![3; 32]).unwrap();
        let a = derive_evidence_digest(
            &spec,
            &closure,
            &[observation("/nix/store/aaaa-git/bin/git")],
        )
        .unwrap();
        let b = derive_evidence_digest(
            &spec,
            &closure,
            &[observation("/nix/store/aaaa-git/bin/git-alt")],
        )
        .unwrap();
        assert_ne!(a, b);
    }
}

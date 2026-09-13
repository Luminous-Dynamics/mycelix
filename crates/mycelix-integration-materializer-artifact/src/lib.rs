//! Hash-pinned local WASM materializer artifact qualification.
//!
//! A signed provider execution profile commits an exact `materializer_release`.
//! This crate resolves that commitment to captured local WASM bytes without
//! granting execution authority, network access, or permission to materialize a
//! provider request.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{ContentCommitment, DigestAlgorithm};
use mycelix_integration_execution_binding::QualifiedProviderExecutionProfile;
use std::fs::{self, OpenOptions};
use std::io::Read;
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use thiserror::Error;

pub const QUALIFICATION_PROFILE: &str =
    "mycelix-integration-materializer-artifact-v1-blake3-framed";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/integration/materializer-artifact/v1";
const WASM_MAGIC_AND_VERSION_V1: &[u8; 8] = b"\0asm\x01\0\0\0";
const MAX_WASM_MODULE_BYTES: u64 = 16 * 1024 * 1024;

/// Non-deserializable captured WASM bytes whose exact SHA-256 equals the release
/// commitment signed into one provider execution profile.
#[derive(Debug)]
pub struct QualifiedWasmMaterializerArtifact {
    module_bytes: Vec<u8>,
    release_commitment: ContentCommitment,
    provider_profile_commitment: ContentCommitment,
    canonical_source: PathBuf,
    source_device: u64,
    source_inode: u64,
    source_mode: u32,
    qualification_digest: Digest32,
}

impl QualifiedWasmMaterializerArtifact {
    pub fn module_bytes(&self) -> &[u8] {
        &self.module_bytes
    }

    pub fn release_commitment(&self) -> &ContentCommitment {
        &self.release_commitment
    }

    pub fn provider_profile_commitment(&self) -> &ContentCommitment {
        &self.provider_profile_commitment
    }

    pub fn canonical_source(&self) -> &Path {
        &self.canonical_source
    }

    pub fn source_device(&self) -> u64 {
        self.source_device
    }

    pub fn source_inode(&self) -> u64 {
        self.source_inode
    }

    pub fn source_mode(&self) -> u32 {
        self.source_mode
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        QUALIFICATION_PROFILE
    }

    pub const fn exact_profile_release_matched_here(&self) -> bool {
        true
    }

    pub const fn wasm_container_verified_here(&self) -> bool {
        true
    }

    pub const fn captured_bytes_immutable_for_object_here(&self) -> bool {
        true
    }

    pub const fn wasm_import_policy_verified_here(&self) -> bool {
        false
    }

    pub const fn deterministic_execution_verified_here(&self) -> bool {
        false
    }

    pub const fn provider_profile_currentness_verified_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Resolve one signed provider profile's exact materializer release to local
/// captured WASM bytes. The provider profile's freshness must be rechecked by the
/// enclosing preexecution theorem at use time; this function proves content
/// identity only.
pub fn qualify_profile_wasm_materializer(
    provider: &QualifiedProviderExecutionProfile,
    path: impl AsRef<Path>,
) -> Result<QualifiedWasmMaterializerArtifact, MaterializerArtifactError> {
    qualify_file_against_release(
        path.as_ref(),
        &provider.profile().materializer_release,
        provider.profile_commitment(),
    )
}

fn qualify_file_against_release(
    path: &Path,
    expected_release: &ContentCommitment,
    provider_profile_commitment: &ContentCommitment,
) -> Result<QualifiedWasmMaterializerArtifact, MaterializerArtifactError> {
    let input_meta = fs::symlink_metadata(path).map_err(MaterializerArtifactError::Io)?;
    if input_meta.file_type().is_symlink() {
        return Err(MaterializerArtifactError::SymlinkArtifact);
    }

    let canonical_source = fs::canonicalize(path).map_err(MaterializerArtifactError::Io)?;
    let mut options = OpenOptions::new();
    options
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC);
    let mut file = options
        .open(&canonical_source)
        .map_err(MaterializerArtifactError::Io)?;
    let before = file.metadata().map_err(MaterializerArtifactError::Io)?;
    validate_metadata(&before)?;

    let mut module_bytes = Vec::with_capacity(
        usize::try_from(before.len()).map_err(|_| MaterializerArtifactError::ArtifactTooLarge)?,
    );
    file.read_to_end(&mut module_bytes)
        .map_err(MaterializerArtifactError::Io)?;
    let after = file.metadata().map_err(MaterializerArtifactError::Io)?;
    if before.dev() != after.dev()
        || before.ino() != after.ino()
        || before.len() != after.len()
        || before.mtime() != after.mtime()
        || before.mtime_nsec() != after.mtime_nsec()
    {
        return Err(MaterializerArtifactError::ArtifactChangedDuringRead);
    }

    validate_wasm_bytes(&module_bytes)?;
    let observed_release = ContentCommitment::sha256(&module_bytes);
    if &observed_release != expected_release {
        return Err(MaterializerArtifactError::ReleaseCommitmentMismatch);
    }

    let source_mode = after.permissions().mode() & 0o7777;
    let qualification_digest = artifact_qualification_digest(
        expected_release,
        provider_profile_commitment,
        &canonical_source,
        after.dev(),
        after.ino(),
        source_mode,
        module_bytes.len(),
    );

    Ok(QualifiedWasmMaterializerArtifact {
        module_bytes,
        release_commitment: expected_release.clone(),
        provider_profile_commitment: provider_profile_commitment.clone(),
        canonical_source,
        source_device: after.dev(),
        source_inode: after.ino(),
        source_mode,
        qualification_digest,
    })
}

fn validate_metadata(metadata: &fs::Metadata) -> Result<(), MaterializerArtifactError> {
    if !metadata.file_type().is_file() {
        return Err(MaterializerArtifactError::NotRegularFile);
    }
    if metadata.len() < WASM_MAGIC_AND_VERSION_V1.len() as u64 {
        return Err(MaterializerArtifactError::InvalidWasmContainer);
    }
    if metadata.len() > MAX_WASM_MODULE_BYTES {
        return Err(MaterializerArtifactError::ArtifactTooLarge);
    }
    let mode = metadata.permissions().mode() & 0o7777;
    if mode & 0o022 != 0 {
        return Err(MaterializerArtifactError::MutableByGroupOrOther);
    }
    Ok(())
}

fn validate_wasm_bytes(bytes: &[u8]) -> Result<(), MaterializerArtifactError> {
    if bytes.len() < WASM_MAGIC_AND_VERSION_V1.len()
        || &bytes[..WASM_MAGIC_AND_VERSION_V1.len()] != WASM_MAGIC_AND_VERSION_V1
    {
        return Err(MaterializerArtifactError::InvalidWasmContainer);
    }
    Ok(())
}

fn artifact_qualification_digest(
    release: &ContentCommitment,
    provider_profile: &ContentCommitment,
    canonical_source: &Path,
    device: u64,
    inode: u64,
    mode: u32,
    byte_len: usize,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame_commitment(&mut h, release);
    frame_commitment(&mut h, provider_profile);
    frame(&mut h, canonical_source.as_os_str().as_encoded_bytes());
    frame(&mut h, &device.to_le_bytes());
    frame(&mut h, &inode.to_le_bytes());
    frame(&mut h, &mode.to_le_bytes());
    frame(&mut h, &(byte_len as u64).to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn frame_commitment(h: &mut blake3::Hasher, commitment: &ContentCommitment) {
    frame(
        h,
        &[match commitment.algorithm {
            DigestAlgorithm::Sha256 => 1,
        }],
    );
    frame(h, &commitment.digest);
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum MaterializerArtifactError {
    #[error("materializer artifact path resolves to a symlink")]
    SymlinkArtifact,
    #[error("materializer artifact is not a regular file")]
    NotRegularFile,
    #[error("materializer artifact exceeds the v0.1 size bound")]
    ArtifactTooLarge,
    #[error("materializer artifact is group/other writable")]
    MutableByGroupOrOther,
    #[error("materializer artifact changed while it was being captured")]
    ArtifactChangedDuringRead,
    #[error("materializer artifact is not a WebAssembly v1 module")]
    InvalidWasmContainer,
    #[error("materializer artifact bytes do not match the signed provider release commitment")]
    ReleaseCommitmentMismatch,
    #[error("materializer artifact filesystem error: {0}")]
    Io(std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::File;
    use std::io::Write;
    use std::os::unix::fs::PermissionsExt;
    use tempfile::tempdir;

    fn minimal_wasm(extra: u8) -> Vec<u8> {
        let mut bytes = WASM_MAGIC_AND_VERSION_V1.to_vec();
        if extra != 0 {
            bytes.extend_from_slice(&[0, 1, extra]);
        }
        bytes
    }

    fn write_artifact(bytes: &[u8]) -> (tempfile::TempDir, PathBuf) {
        let temp = tempdir().unwrap();
        let path = temp.path().join("materializer.wasm");
        let mut file = File::create(&path).unwrap();
        file.write_all(bytes).unwrap();
        drop(file);
        fs::set_permissions(&path, fs::Permissions::from_mode(0o555)).unwrap();
        (temp, path)
    }

    #[test]
    fn exact_hash_pinned_wasm_is_captured() {
        let bytes = minimal_wasm(0);
        let (_temp, path) = write_artifact(&bytes);
        let release = ContentCommitment::sha256(&bytes);
        let profile = ContentCommitment::sha256(b"provider-profile");
        let qualified = qualify_file_against_release(&path, &release, &profile).unwrap();
        assert_eq!(qualified.module_bytes(), bytes.as_slice());
        assert_eq!(qualified.release_commitment(), &release);
        assert!(qualified.exact_profile_release_matched_here());
        assert!(qualified.wasm_container_verified_here());
        assert!(qualified.captured_bytes_immutable_for_object_here());
        assert!(!qualified.wasm_import_policy_verified_here());
        assert!(!qualified.deterministic_execution_verified_here());
        assert!(!qualified.grants_execution_authority());
    }

    #[test]
    fn release_substitution_fails_closed() {
        let bytes = minimal_wasm(0);
        let (_temp, path) = write_artifact(&bytes);
        let wrong = ContentCommitment::sha256(b"different-release");
        let profile = ContentCommitment::sha256(b"provider-profile");
        assert!(matches!(
            qualify_file_against_release(&path, &wrong, &profile),
            Err(MaterializerArtifactError::ReleaseCommitmentMismatch)
        ));
    }

    #[test]
    fn native_or_arbitrary_bytes_are_rejected() {
        let bytes = b"#!/bin/sh\necho nope\n".to_vec();
        let (_temp, path) = write_artifact(&bytes);
        let release = ContentCommitment::sha256(&bytes);
        let profile = ContentCommitment::sha256(b"provider-profile");
        assert!(matches!(
            qualify_file_against_release(&path, &release, &profile),
            Err(MaterializerArtifactError::InvalidWasmContainer)
        ));
    }

    #[test]
    fn writable_artifact_is_rejected() {
        let bytes = minimal_wasm(0);
        let (temp, path) = write_artifact(&bytes);
        fs::set_permissions(&path, fs::Permissions::from_mode(0o577)).unwrap();
        let release = ContentCommitment::sha256(&bytes);
        let profile = ContentCommitment::sha256(b"provider-profile");
        assert!(matches!(
            qualify_file_against_release(&path, &release, &profile),
            Err(MaterializerArtifactError::MutableByGroupOrOther)
        ));
        drop(temp);
    }

    #[test]
    fn symlink_artifact_is_rejected() {
        use std::os::unix::fs::symlink;

        let bytes = minimal_wasm(0);
        let (temp, target) = write_artifact(&bytes);
        let link = temp.path().join("materializer-link.wasm");
        symlink(&target, &link).unwrap();
        let release = ContentCommitment::sha256(&bytes);
        let profile = ContentCommitment::sha256(b"provider-profile");
        assert!(matches!(
            qualify_file_against_release(&link, &release, &profile),
            Err(MaterializerArtifactError::SymlinkArtifact)
        ));
    }
}

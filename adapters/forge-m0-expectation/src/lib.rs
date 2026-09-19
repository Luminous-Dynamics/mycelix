// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D4I: canonical external expectation handoff for Forge M0.
//!
//! This crate turns the four retained D4E subjects needed by D4G/D4H into one
//! versioned portable artifact. It is a handoff contract, not an authority
//! grant. The file is intended to be retained/transferred separately from the
//! materialized capsule it will later authenticate.

use mycelix_forge_core::{Digest, DigestAlgorithm, ForgeCoreError};
use mycelix_forge_m0_capsule_builder::BuiltM0Capsule;
use serde::{Deserialize, Serialize};
use std::{
    ffi::CString,
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    os::unix::{
        ffi::OsStrExt,
        fs::{OpenOptionsExt, PermissionsExt},
    },
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};
use thiserror::Error;

const EXPECTATION_DOMAIN_V1: &[u8] = b"mycelix-forge/m0-expectation/v1\0";
pub const M0_EXPECTATION_SCHEMA_VERSION: u16 = 1;
const FILE_MODE: u32 = 0o400;
const MAX_EXPECTATION_BYTES: usize = 64 * 1024;
const MAX_STAGE_ATTEMPTS: usize = 32;
static STAGING_COUNTER: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct M0ExpectationV1 {
    schema_version: u16,
    construction_commitment: Digest,
    execution_subject: Digest,
    execution_spec: Digest,
    run_plan: Digest,
}

impl M0ExpectationV1 {
    pub fn new(
        construction_commitment: Digest,
        execution_subject: Digest,
        execution_spec: Digest,
        run_plan: Digest,
    ) -> Result<Self, ExpectationError> {
        let value = Self {
            schema_version: M0_EXPECTATION_SCHEMA_VERSION,
            construction_commitment,
            execution_subject,
            execution_spec,
            run_plan,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn from_capsule(capsule: &BuiltM0Capsule) -> Result<Self, ExpectationError> {
        let execution_spec = capsule
            .execution_spec()
            .digest(DigestAlgorithm::Sha256)
            .map_err(|error| ExpectationError::SubjectDerivation(error.to_string()))?;
        let run_plan = capsule
            .run_plan()
            .digest(DigestAlgorithm::Sha256)
            .map_err(|error| ExpectationError::SubjectDerivation(error.to_string()))?;
        Self::new(
            capsule.construction_commitment().clone(),
            capsule.execution_subject().clone(),
            execution_spec,
            run_plan,
        )
    }

    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub fn construction_commitment(&self) -> &Digest {
        &self.construction_commitment
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn execution_spec(&self) -> &Digest {
        &self.execution_spec
    }

    pub fn run_plan(&self) -> &Digest {
        &self.run_plan
    }

    pub fn validate(&self) -> Result<(), ExpectationError> {
        if self.schema_version != M0_EXPECTATION_SCHEMA_VERSION {
            return Err(ExpectationError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        for (field, digest) in [
            ("construction_commitment", &self.construction_commitment),
            ("execution_subject", &self.execution_subject),
            ("execution_spec", &self.execution_spec),
            ("run_plan", &self.run_plan),
        ] {
            if digest.algorithm() != DigestAlgorithm::Sha256 {
                return Err(ExpectationError::UnexpectedDigestAlgorithm(field));
            }
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ExpectationError> {
        self.validate()?;
        let mut out = Vec::new();
        out.extend_from_slice(EXPECTATION_DOMAIN_V1);
        out.extend_from_slice(&self.schema_version.to_be_bytes());
        for digest in [
            &self.construction_commitment,
            &self.execution_subject,
            &self.execution_spec,
            &self.run_plan,
        ] {
            push_digest(&mut out, digest)?;
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ExpectationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }

    pub fn json_bytes(&self) -> Result<Vec<u8>, ExpectationError> {
        self.validate()?;
        let bytes = serde_json::to_vec(self)?;
        if bytes.len() > MAX_EXPECTATION_BYTES {
            return Err(ExpectationError::ExpectationTooLarge);
        }
        Ok(bytes)
    }

    pub fn transport_digest(&self) -> Result<Digest, ExpectationError> {
        Ok(Digest::of_bytes(
            DigestAlgorithm::Sha256,
            &self.json_bytes()?,
        ))
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PublishedM0Expectation {
    path: PathBuf,
    semantic_digest: Digest,
    transport_digest: Digest,
    size: u64,
}

impl PublishedM0Expectation {
    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn semantic_digest(&self) -> &Digest {
        &self.semantic_digest
    }

    pub fn transport_digest(&self) -> &Digest {
        &self.transport_digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }
}

/// Atomically publish one expectation file with no-replace semantics.
///
/// The parent directory is resolved before staging. This function does not
/// claim resistance to a malicious privileged host mutating that directory;
/// the same trusted-host boundary as M0 applies.
pub fn publish_expectation_noreplace(
    expectation: &M0ExpectationV1,
    destination: &Path,
) -> Result<PublishedM0Expectation, ExpectationError> {
    let bytes = expectation.json_bytes()?;
    let size = u64::try_from(bytes.len()).map_err(|_| ExpectationError::ExpectationTooLarge)?;
    let semantic_digest = expectation.digest(DigestAlgorithm::Sha256)?;
    let transport_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &bytes);

    let final_path = normalize_new_path(destination)?;
    let parent = final_path.parent().ok_or(ExpectationError::InvalidDestination)?;
    let (staging, mut file) = create_staging_file(parent)?;
    let mut guard = StagingGuard::new(staging.clone());

    file.write_all(&bytes)?;
    file.sync_all()?;
    file.set_permissions(fs::Permissions::from_mode(FILE_MODE))?;
    file.sync_all()?;
    drop(file);

    verify_file(&staging, expectation, &bytes)?;
    rename_noreplace(&staging, &final_path)?;
    guard.disarm();
    File::open(parent)?.sync_all()?;
    verify_file(&final_path, expectation, &bytes)?;

    Ok(PublishedM0Expectation {
        path: final_path,
        semantic_digest,
        transport_digest,
        size,
    })
}

pub fn load_expectation_file(path: &Path) -> Result<(M0ExpectationV1, Digest), ExpectationError> {
    let bytes = read_bounded_nofollow(path)?;
    let transport_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &bytes);
    let expectation: M0ExpectationV1 = serde_json::from_slice(&bytes)?;
    expectation.validate()?;
    Ok((expectation, transport_digest))
}

fn verify_file(
    path: &Path,
    expectation: &M0ExpectationV1,
    expected_bytes: &[u8],
) -> Result<(), ExpectationError> {
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(ExpectationError::InvalidExpectationFile(path.to_path_buf()));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != FILE_MODE {
        return Err(ExpectationError::UnexpectedFileMode {
            path: path.to_path_buf(),
            mode,
        });
    }
    let bytes = read_bounded_nofollow(path)?;
    if bytes != expected_bytes {
        return Err(ExpectationError::PublishedBytesMismatch);
    }
    let parsed: M0ExpectationV1 = serde_json::from_slice(&bytes)?;
    parsed.validate()?;
    if &parsed != expectation
        || parsed.digest(DigestAlgorithm::Sha256)?
            != expectation.digest(DigestAlgorithm::Sha256)?
    {
        return Err(ExpectationError::PublishedSemanticMismatch);
    }
    Ok(())
}

fn read_bounded_nofollow(path: &Path) -> Result<Vec<u8>, ExpectationError> {
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(ExpectationError::InvalidExpectationFile(path.to_path_buf()));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode & 0o022 != 0 {
        return Err(ExpectationError::ExpectationWritableByOthers {
            path: path.to_path_buf(),
            mode,
        });
    }
    let mut file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?;
    let limit = u64::try_from(MAX_EXPECTATION_BYTES)
        .map_err(|_| ExpectationError::ExpectationTooLarge)?
        .saturating_add(1);
    let mut bytes = Vec::new();
    file.by_ref().take(limit).read_to_end(&mut bytes)?;
    if bytes.len() > MAX_EXPECTATION_BYTES {
        return Err(ExpectationError::ExpectationTooLarge);
    }
    Ok(bytes)
}

fn normalize_new_path(destination: &Path) -> Result<PathBuf, ExpectationError> {
    let absolute = if destination.is_absolute() {
        destination.to_path_buf()
    } else {
        std::env::current_dir()?.join(destination)
    };
    let name = absolute
        .file_name()
        .ok_or(ExpectationError::InvalidDestination)?;
    let parent = fs::canonicalize(
        absolute
            .parent()
            .ok_or(ExpectationError::InvalidDestination)?,
    )?;
    if !fs::metadata(&parent)?.is_dir() {
        return Err(ExpectationError::InvalidDestination);
    }
    let final_path = parent.join(name);
    match fs::symlink_metadata(&final_path) {
        Ok(_) => Err(ExpectationError::DestinationExists(final_path)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(final_path),
        Err(error) => Err(error.into()),
    }
}

fn create_staging_file(parent: &Path) -> Result<(PathBuf, File), ExpectationError> {
    for _ in 0..MAX_STAGE_ATTEMPTS {
        let sequence = STAGING_COUNTER.fetch_add(1, Ordering::Relaxed);
        let candidate = parent.join(format!(
            ".mycelix-forge-expectation-stage-{}-{sequence}",
            std::process::id()
        ));
        match OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&candidate)
        {
            Ok(file) => return Ok((candidate, file)),
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(error.into()),
        }
    }
    Err(ExpectationError::UnableToCreateStagingFile)
}

fn rename_noreplace(source: &Path, destination: &Path) -> Result<(), ExpectationError> {
    let source = c_path(source)?;
    let destination_c = c_path(destination)?;
    let result = unsafe {
        libc::syscall(
            libc::SYS_renameat2,
            libc::AT_FDCWD,
            source.as_ptr(),
            libc::AT_FDCWD,
            destination_c.as_ptr(),
            libc::RENAME_NOREPLACE,
        )
    };
    if result != 0 {
        let error = std::io::Error::last_os_error();
        if error.raw_os_error() == Some(libc::EEXIST) {
            return Err(ExpectationError::DestinationExists(destination.to_path_buf()));
        }
        return Err(error.into());
    }
    Ok(())
}

fn c_path(path: &Path) -> Result<CString, ExpectationError> {
    CString::new(path.as_os_str().as_bytes())
        .map_err(|_| ExpectationError::PathContainsNul(path.to_path_buf()))
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ExpectationError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| ExpectationError::CanonicalLengthOverflow)?;
    let digest_len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| ExpectationError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

struct StagingGuard {
    path: PathBuf,
    armed: bool,
}

impl StagingGuard {
    fn new(path: PathBuf) -> Self {
        Self { path, armed: true }
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for StagingGuard {
    fn drop(&mut self) {
        if self.armed {
            let _ = fs::remove_file(&self.path);
        }
    }
}

#[derive(Debug, Error)]
pub enum ExpectationError {
    #[error(transparent)]
    Core(#[from] ForgeCoreError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("unsupported M0 expectation schema version: {0}")]
    UnsupportedSchemaVersion(u16),
    #[error("expectation field {0} does not use the exact M0 SHA-256 profile")]
    UnexpectedDigestAlgorithm(&'static str),
    #[error("failed to derive expectation subject: {0}")]
    SubjectDerivation(String),
    #[error("expectation JSON exceeds the 64 KiB transport limit")]
    ExpectationTooLarge,
    #[error("invalid expectation destination")]
    InvalidDestination,
    #[error("expectation destination already exists: {0:?}")]
    DestinationExists(PathBuf),
    #[error("unable to allocate an expectation staging file")]
    UnableToCreateStagingFile,
    #[error("expectation path contains NUL: {0:?}")]
    PathContainsNul(PathBuf),
    #[error("invalid expectation file: {0:?}")]
    InvalidExpectationFile(PathBuf),
    #[error("expectation file is writable by group/other: {path:?} mode={mode:o}")]
    ExpectationWritableByOthers { path: PathBuf, mode: u32 },
    #[error("published expectation file mode differs from 0400: {path:?} mode={mode:o}")]
    UnexpectedFileMode { path: PathBuf, mode: u32 },
    #[error("published expectation bytes differ from constructed bytes")]
    PublishedBytesMismatch,
    #[error("published expectation semantic subject differs after reparse")]
    PublishedSemanticMismatch,
    #[error("canonical expectation field length overflow")]
    CanonicalLengthOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn expectation() -> M0ExpectationV1 {
        M0ExpectationV1::new(digest(1), digest(2), digest(3), digest(4)).unwrap()
    }

    fn test_path(name: &str) -> PathBuf {
        let sequence = STAGING_COUNTER.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!(
            "mycelix-forge-expectation-{name}-{}-{sequence}.json",
            std::process::id()
        ))
    }

    #[test]
    fn canonical_digest_is_stable_across_json_round_trip() {
        let original = expectation();
        let parsed: M0ExpectationV1 =
            serde_json::from_slice(&original.json_bytes().unwrap()).unwrap();
        assert_eq!(original, parsed);
        assert_eq!(
            original.digest(DigestAlgorithm::Sha256).unwrap(),
            parsed.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn changing_any_subject_changes_semantic_digest() {
        let original = expectation();
        let changed = M0ExpectationV1::new(digest(1), digest(2), digest(3), digest(5)).unwrap();
        assert_ne!(
            original.digest(DigestAlgorithm::Sha256).unwrap(),
            changed.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn publication_is_no_replace_and_owner_read_only() {
        let path = test_path("publish");
        let original = expectation();
        let published = publish_expectation_noreplace(&original, &path).unwrap();
        assert_eq!(published.path(), path.as_path());
        assert_eq!(
            fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            FILE_MODE
        );
        let (loaded, transport) = load_expectation_file(&path).unwrap();
        assert_eq!(loaded, original);
        assert_eq!(&transport, published.transport_digest());
        assert!(matches!(
            publish_expectation_noreplace(&original, &path),
            Err(ExpectationError::DestinationExists(_))
        ));
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn non_sha256_subject_is_rejected() {
        let blake = Digest::new(DigestAlgorithm::Blake3_256, vec![9; 32]).unwrap();
        assert!(matches!(
            M0ExpectationV1::new(blake, digest(2), digest(3), digest(4)),
            Err(ExpectationError::UnexpectedDigestAlgorithm(
                "construction_commitment"
            ))
        ));
    }
}

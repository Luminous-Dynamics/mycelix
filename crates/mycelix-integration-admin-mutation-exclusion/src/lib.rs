// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Shared coordinator Admin-mutation exclusion interval for exact integration admissions.
//!
//! This adapter intentionally reuses #539's public lock binding and constants,
//! plus the same lower-level Admin isolation theorem. It does not expose an Admin
//! endpoint, provider client, payload materializer, or dispatch transition.

#[cfg(not(target_os = "linux"))]
compile_error!("integration Admin-mutation exclusion v0.1 requires Linux");

use mycelix_authority_coordinator_admin_isolation::{
    begin_exclusive_admin_isolation, AdminIsolationError, BrokerProcessBinding,
    ExclusiveAdminIsolationGuard, QualifiedExclusiveAdminIsolation,
};
use mycelix_authority_coordinator_admin_mutation_exclusion::{
    AdminMutationExclusionBinding, AdminMutationExclusionError, LOCK_DIR_MODE, LOCK_FILE_MODE,
    LOCK_PATH,
};
use mycelix_authority_coordinator_conductor_process::TrustedConductorProcessStore;
use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{ContentCommitment, DigestAlgorithm, ExecutionAttemptId};
use mycelix_integration_store_rechecked_preexecution::QualifiedStoreRecheckedPreexecutionAdmission;
use std::fs::{self, File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};
use thiserror::Error;

pub const SUBJECT_PROFILE: &str =
    "mycelix-integration-admin-mutation-exclusion-subject-v1-blake3-framed";
pub const INTERVAL_PROFILE: &str =
    "mycelix-integration-admin-mutation-exclusion-interval-v1-blake3-framed";

const DOMAIN_SUBJECT: &[u8] = b"mycelix/integration/admin-mutation-exclusion/subject/v1";
const DOMAIN_INTERVAL: &[u8] = b"mycelix/integration/admin-mutation-exclusion/interval/v1";

/// Live non-serializable guard proving the exact #539 lock domain and lower-level
/// Admin isolation are both held for one integration admission subject.
pub struct IntegrationAdminMutationExclusionGuard<'a> {
    lock: ExclusiveIntegrationMutationLock,
    isolation: ExclusiveAdminIsolationGuard<'a>,
    binding: AdminMutationExclusionBinding,
    binding_digest: [u8; 32],
    subject_digest: Digest32,
    entry_id: i64,
    attempt_id: ExecutionAttemptId,
    command_commitment: ContentCommitment,
    started_at_ms: u64,
}

impl IntegrationAdminMutationExclusionGuard<'_> {
    pub fn exclusion_binding_digest(&self) -> [u8; 32] {
        self.binding_digest
    }

    pub fn subject_digest(&self) -> Digest32 {
        self.subject_digest
    }

    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &ExecutionAttemptId {
        &self.attempt_id
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        &self.command_commitment
    }

    pub fn started_at_ms(&self) -> u64 {
        self.started_at_ms
    }

    pub const fn shared_admin_mutation_lock_held_here(&self) -> bool {
        true
    }

    pub const fn admin_isolation_held_here(&self) -> bool {
        true
    }

    pub const fn exact_integration_subject_bound_here(&self) -> bool {
        true
    }

    pub const fn preexecution_requalified_inside_guard_here(&self) -> bool {
        false
    }

    pub const fn provider_root_rechecked_inside_guard_here(&self) -> bool {
        false
    }

    pub const fn dispatch_started_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn provider_call_started_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }

    /// Close the exact lower-level isolation interval while the shared #539 lock
    /// is still held and revalidated. The resulting proof is historical only: it
    /// does not prove what arbitrary caller work happened inside the interval.
    pub fn finish(
        self,
    ) -> Result<QualifiedIntegrationAdminMutationExclusion, IntegrationExclusionError> {
        let Self {
            lock,
            isolation,
            binding,
            binding_digest,
            subject_digest,
            entry_id,
            attempt_id,
            command_commitment,
            started_at_ms,
        } = self;

        lock.validate(&binding)?;
        let isolation = isolation.finish().map_err(IntegrationExclusionError::AdminIsolation)?;
        lock.validate(&binding)?;
        let ended_at_ms = now_ms()?;
        if ended_at_ms < started_at_ms || isolation.ended_at_ms() > ended_at_ms {
            return Err(IntegrationExclusionError::InvalidInterval);
        }

        let interval_digest = interval_digest(
            binding_digest,
            subject_digest,
            entry_id,
            &attempt_id,
            &command_commitment,
            &isolation,
            started_at_ms,
            ended_at_ms,
        );

        Ok(QualifiedIntegrationAdminMutationExclusion {
            exclusion_binding_digest: binding_digest,
            subject_digest,
            entry_id,
            attempt_id,
            command_commitment,
            isolation,
            interval_digest,
            started_at_ms,
            ended_at_ms,
        })
    }
}

/// Historical proof that the exact integration subject occupied the same pinned
/// coordinator-mutation exclusion domain used by #539 for this interval.
#[derive(Clone, Debug)]
pub struct QualifiedIntegrationAdminMutationExclusion {
    exclusion_binding_digest: [u8; 32],
    subject_digest: Digest32,
    entry_id: i64,
    attempt_id: ExecutionAttemptId,
    command_commitment: ContentCommitment,
    isolation: QualifiedExclusiveAdminIsolation,
    interval_digest: Digest32,
    started_at_ms: u64,
    ended_at_ms: u64,
}

impl QualifiedIntegrationAdminMutationExclusion {
    pub fn exclusion_binding_digest(&self) -> [u8; 32] {
        self.exclusion_binding_digest
    }

    pub fn subject_digest(&self) -> Digest32 {
        self.subject_digest
    }

    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &ExecutionAttemptId {
        &self.attempt_id
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        &self.command_commitment
    }

    pub fn isolation(&self) -> &QualifiedExclusiveAdminIsolation {
        &self.isolation
    }

    pub fn interval_digest(&self) -> Digest32 {
        self.interval_digest
    }

    pub fn interval_profile(&self) -> &'static str {
        INTERVAL_PROFILE
    }

    pub fn started_at_ms(&self) -> u64 {
        self.started_at_ms
    }

    pub fn ended_at_ms(&self) -> u64 {
        self.ended_at_ms
    }

    pub const fn coordinator_update_excluded_for_interval_here(&self) -> bool {
        true
    }

    pub const fn exact_integration_subject_bound_here(&self) -> bool {
        true
    }

    pub const fn work_inside_interval_verified_here(&self) -> bool {
        false
    }

    pub const fn dispatch_started_here(&self) -> bool {
        false
    }

    pub const fn external_effect_started_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Enter the exact same pinned `flock` + lower-level Admin isolation domain used
/// by #539, binding the live interval to one exact store-rechecked integration
/// admission. This function itself does not re-run the admission inside the lock.
pub fn begin_integration_admin_mutation_exclusion<'a>(
    binding: &AdminMutationExclusionBinding,
    conductor_store: &'a TrustedConductorProcessStore,
    broker_binding: &BrokerProcessBinding,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<IntegrationAdminMutationExclusionGuard<'a>, IntegrationExclusionError> {
    binding.validate()?;
    let binding_digest = binding.binding_digest()?;
    let broker_digest = broker_binding
        .binding_digest()
        .map_err(IntegrationExclusionError::AdminIsolation)?;
    if broker_digest != binding.broker_binding_digest {
        return Err(IntegrationExclusionError::BrokerBindingMismatch);
    }

    let execution = admission.admission().execution_binding();
    let entry_id = execution.entry_id();
    if entry_id <= 0 {
        return Err(IntegrationExclusionError::InvalidIntegrationSubject);
    }
    let attempt_id = execution.attempt_id().clone();
    let command_commitment = execution.command_commitment().clone();
    let subject_digest = integration_subject_digest(admission);
    if subject_digest.is_zero() {
        return Err(IntegrationExclusionError::InvalidIntegrationSubject);
    }

    // Preserve #539's lock-before-isolation ordering so coordinator mutation and
    // integration effect intervals share one serialization theorem.
    let lock = acquire_shared_lock(binding)?;
    let isolation = begin_exclusive_admin_isolation(conductor_store, broker_binding)
        .map_err(IntegrationExclusionError::AdminIsolation)?;
    lock.validate(binding)?;
    let started_at_ms = now_ms()?;

    Ok(IntegrationAdminMutationExclusionGuard {
        lock,
        isolation,
        binding: binding.clone(),
        binding_digest,
        subject_digest,
        entry_id,
        attempt_id,
        command_commitment,
        started_at_ms,
    })
}

fn integration_subject_digest(admission: &QualifiedStoreRecheckedPreexecutionAdmission) -> Digest32 {
    let execution = admission.admission().execution_binding();
    integration_subject_digest_from_parts(
        admission.qualification_digest(),
        admission.current_attempt().store_state_digest(),
        execution.entry_id(),
        execution.attempt_id(),
        execution.command_commitment(),
    )
}

fn integration_subject_digest_from_parts(
    admission_digest: Digest32,
    store_state_digest: [u8; 32],
    entry_id: i64,
    attempt_id: &ExecutionAttemptId,
    command_commitment: &ContentCommitment,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_SUBJECT);
    frame(&mut h, SUBJECT_PROFILE.as_bytes());
    frame(&mut h, &admission_digest.0);
    frame(&mut h, &store_state_digest);
    frame(&mut h, &entry_id.to_le_bytes());
    frame(&mut h, attempt_id.as_str().as_bytes());
    frame_commitment(&mut h, command_commitment);
    Digest32(*h.finalize().as_bytes())
}

#[allow(clippy::too_many_arguments)]
fn interval_digest(
    binding_digest: [u8; 32],
    subject_digest: Digest32,
    entry_id: i64,
    attempt_id: &ExecutionAttemptId,
    command_commitment: &ContentCommitment,
    isolation: &QualifiedExclusiveAdminIsolation,
    started_at_ms: u64,
    ended_at_ms: u64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_INTERVAL);
    frame(&mut h, INTERVAL_PROFILE.as_bytes());
    frame(&mut h, &binding_digest);
    frame(&mut h, &subject_digest.0);
    frame(&mut h, &entry_id.to_le_bytes());
    frame(&mut h, attempt_id.as_str().as_bytes());
    frame_commitment(&mut h, command_commitment);
    frame(&mut h, &isolation.isolation_digest());
    frame(&mut h, isolation.isolation_profile().as_bytes());
    frame(&mut h, &started_at_ms.to_le_bytes());
    frame(&mut h, &ended_at_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

struct ExclusiveIntegrationMutationLock {
    file: File,
}

impl ExclusiveIntegrationMutationLock {
    fn validate(
        &self,
        binding: &AdminMutationExclusionBinding,
    ) -> Result<(), IntegrationExclusionError> {
        validate_lock_file(&self.file, binding)
    }
}

fn acquire_shared_lock(
    binding: &AdminMutationExclusionBinding,
) -> Result<ExclusiveIntegrationMutationLock, IntegrationExclusionError> {
    validate_lock_parent()?;
    let path = Path::new(LOCK_PATH);
    let mut options = OpenOptions::new();
    options
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC);
    let file = options.open(path).map_err(IntegrationExclusionError::Io)?;
    validate_lock_file(&file, binding)?;

    let rc = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX) };
    if rc != 0 {
        return Err(IntegrationExclusionError::Io(io::Error::last_os_error()));
    }
    validate_lock_file(&file, binding)?;
    Ok(ExclusiveIntegrationMutationLock { file })
}

fn validate_lock_parent() -> Result<(), IntegrationExclusionError> {
    let path = Path::new(LOCK_PATH);
    let parent = path.parent().ok_or(IntegrationExclusionError::InvalidLockPath)?;
    if !parent.is_absolute() {
        return Err(IntegrationExclusionError::InvalidLockPath);
    }
    let canonical = fs::canonicalize(parent).map_err(IntegrationExclusionError::Io)?;
    if canonical != PathBuf::from(parent) {
        return Err(IntegrationExclusionError::NonCanonicalLockDirectory);
    }
    let metadata = fs::symlink_metadata(parent).map_err(IntegrationExclusionError::Io)?;
    if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
        return Err(IntegrationExclusionError::UnsafeLockDirectory);
    }
    let euid = unsafe { libc::geteuid() };
    if metadata.uid() != euid {
        return Err(IntegrationExclusionError::LockOwnerMismatch);
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != LOCK_DIR_MODE {
        return Err(IntegrationExclusionError::UnsafeLockDirectoryMode {
            expected: LOCK_DIR_MODE,
            observed: mode,
        });
    }
    Ok(())
}

fn validate_lock_file(
    file: &File,
    binding: &AdminMutationExclusionBinding,
) -> Result<(), IntegrationExclusionError> {
    let metadata = file.metadata().map_err(IntegrationExclusionError::Io)?;
    if !metadata.file_type().is_file() {
        return Err(IntegrationExclusionError::UnsafeLockFile);
    }
    let euid = unsafe { libc::geteuid() };
    if metadata.uid() != euid {
        return Err(IntegrationExclusionError::LockOwnerMismatch);
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != LOCK_FILE_MODE {
        return Err(IntegrationExclusionError::UnsafeLockFileMode {
            expected: LOCK_FILE_MODE,
            observed: mode,
        });
    }
    if metadata.dev() != binding.lock_device || metadata.ino() != binding.lock_inode {
        return Err(IntegrationExclusionError::LockIdentityMismatch);
    }
    Ok(())
}

fn now_ms() -> Result<u64, IntegrationExclusionError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| IntegrationExclusionError::ClockBeforeUnixEpoch)?;
    let value = u64::try_from(duration.as_millis()).map_err(|_| IntegrationExclusionError::ClockOverflow)?;
    if value == 0 {
        Err(IntegrationExclusionError::ClockBeforeUnixEpoch)
    } else {
        Ok(value)
    }
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
pub enum IntegrationExclusionError {
    #[error("integration exclusion subject is invalid")]
    InvalidIntegrationSubject,
    #[error("broker binding does not match the #539 exclusion binding")]
    BrokerBindingMismatch,
    #[error("invalid fixed Admin mutation lock path")]
    InvalidLockPath,
    #[error("Admin mutation lock directory path is not canonical")]
    NonCanonicalLockDirectory,
    #[error("Admin mutation lock parent is not a safe directory")]
    UnsafeLockDirectory,
    #[error("unsafe Admin mutation lock directory mode: expected {expected:o}, observed {observed:o}")]
    UnsafeLockDirectoryMode { expected: u32, observed: u32 },
    #[error("Admin mutation lock is not a regular file")]
    UnsafeLockFile,
    #[error("unsafe Admin mutation lock file mode: expected {expected:o}, observed {observed:o}")]
    UnsafeLockFileMode { expected: u32, observed: u32 },
    #[error("Admin mutation lock ownership mismatch")]
    LockOwnerMismatch,
    #[error("Admin mutation lock device/inode differs from the #539 binding")]
    LockIdentityMismatch,
    #[error("invalid Admin mutation exclusion interval")]
    InvalidInterval,
    #[error("system clock is before Unix epoch")]
    ClockBeforeUnixEpoch,
    #[error("system clock does not fit u64 milliseconds")]
    ClockOverflow,
    #[error("Admin mutation exclusion filesystem error: {0}")]
    Io(std::io::Error),
    #[error(transparent)]
    AdminIsolation(#[from] AdminIsolationError),
    #[error(transparent)]
    UpstreamBinding(#[from] AdminMutationExclusionError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_integration_core::ValidationError;

    fn iid<T>(value: &str, constructor: fn(String) -> Result<T, ValidationError>) -> T {
        constructor(value.to_owned()).unwrap()
    }

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn c(byte: u8) -> ContentCommitment {
        ContentCommitment::sha256(&[byte])
    }

    #[test]
    fn exact_subject_identity_is_stable() {
        let attempt = iid("1:1", ExecutionAttemptId::new);
        let first = integration_subject_digest_from_parts(d(1), [2; 32], 1, &attempt, &c(3));
        let second = integration_subject_digest_from_parts(d(1), [2; 32], 1, &attempt, &c(3));
        assert_eq!(first, second);
        assert!(!first.is_zero());
    }

    #[test]
    fn durable_store_state_changes_subject_identity() {
        let attempt = iid("1:1", ExecutionAttemptId::new);
        let first = integration_subject_digest_from_parts(d(1), [2; 32], 1, &attempt, &c(3));
        let second = integration_subject_digest_from_parts(d(1), [9; 32], 1, &attempt, &c(3));
        assert_ne!(first, second);
    }

    #[test]
    fn attempt_fence_changes_subject_identity() {
        let first_attempt = iid("1:1", ExecutionAttemptId::new);
        let second_attempt = iid("1:2", ExecutionAttemptId::new);
        let first = integration_subject_digest_from_parts(d(1), [2; 32], 1, &first_attempt, &c(3));
        let second = integration_subject_digest_from_parts(d(1), [2; 32], 1, &second_attempt, &c(3));
        assert_ne!(first, second);
    }

    #[test]
    fn command_changes_subject_identity() {
        let attempt = iid("1:1", ExecutionAttemptId::new);
        let first = integration_subject_digest_from_parts(d(1), [2; 32], 1, &attempt, &c(3));
        let second = integration_subject_digest_from_parts(d(1), [2; 32], 1, &attempt, &c(4));
        assert_ne!(first, second);
    }
}

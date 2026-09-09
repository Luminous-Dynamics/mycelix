// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Shared Linux exclusion boundary for Holochain Admin mutation and Mycelix
//! response critical sections.
//!
//! #535 makes the Admin endpoint reachable only from one pinned broker +
//! conductor network namespace under its stated Linux/root assumptions. This
//! crate adds the missing serialization theorem inside that broker boundary:
//! coordinator updates and response critical sections acquire the same pinned
//! `flock(LOCK_EX)` file before opening the #535 isolation interval.
//!
//! The lock file is provisioned out of band and its exact device+inode are part
//! of the exclusion binding. A replaced lock path therefore cannot silently
//! create a second exclusion domain.
//!
//! No external response effect is performed here. The response guard merely
//! provides a caller-controlled interval during which coordinator update is
//! excluded. A later native orchestrator must own the exact work performed in
//! that interval before it may claim effect admission.

#[cfg(not(target_os = "linux"))]
compile_error!("coordinator Admin mutation exclusion v0.1 requires Linux");

use holochain_client::AdminWebsocket;
use holochain_types::prelude::UpdateCoordinatorsPayload;
use mycelix_authority_coordinator_admin_isolation::{
    begin_exclusive_admin_isolation, AdminIsolationError, BrokerProcessBinding,
    ExclusiveAdminIsolationGuard, QualifiedExclusiveAdminIsolation,
};
use mycelix_authority_coordinator_conductor_process::TrustedConductorProcessStore;
use mycelix_institutional_core::Digest32;
use mycelix_response_attempt_store::DurablyReservedResponseAttempt;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-admin-mutation-exclusion-v0.1";
pub const BINDING_PROFILE: &str =
    "mycelix-authority-coordinator-admin-mutation-exclusion-binding-v1-blake3-framed";
pub const RESPONSE_EXCLUSION_PROFILE: &str =
    "mycelix-response-admin-mutation-exclusion-v1-blake3-framed";
pub const COORDINATOR_UPDATE_PROFILE: &str =
    "mycelix-authority-coordinator-update-operation-v1-blake3-framed";
pub const LOCK_PATH: &str = "/var/lib/mycelix/authority/admin-mutation.lock";
pub const LOCK_FILE_MODE: u32 = 0o600;
pub const LOCK_DIR_MODE: u32 = 0o700;

const BROKER_ORIGIN: &str = "mycelix-authority-admin-mutation-broker-v0.1";
const DOMAIN_BINDING: &[u8] = b"mycelix/authority/admin-mutation-exclusion/binding/v1";
const DOMAIN_RESPONSE: &[u8] = b"mycelix/response/admin-mutation-exclusion/v1";
const DOMAIN_UPDATE: &[u8] = b"mycelix/authority/coordinator-update-operation/v1";
const MAX_TEXT_BYTES: usize = 2048;

/// Out-of-band pin for the one Admin mutation exclusion domain.
///
/// The lock file MUST already exist at `LOCK_PATH`. Its exact device/inode are
/// provisioned into this binding so path replacement cannot create split-brain
/// locks under the same logical configuration.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdminMutationExclusionBinding {
    pub protocol_version: String,
    pub broker_binding_digest: [u8; 32],
    pub lock_device: u64,
    pub lock_inode: u64,
    pub binding_ref: String,
}

impl AdminMutationExclusionBinding {
    pub fn validate(&self) -> Result<(), AdminMutationExclusionError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(AdminMutationExclusionError::WrongProtocol);
        }
        validate_digest(&self.broker_binding_digest, "broker binding digest")?;
        if self.lock_device == 0 || self.lock_inode == 0 {
            return Err(AdminMutationExclusionError::InvalidLockIdentity);
        }
        validate_text(&self.binding_ref, "exclusion binding ref")?;
        Ok(())
    }

    pub fn binding_digest(&self) -> Result<[u8; 32], AdminMutationExclusionError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_BINDING);
        frame(&mut h, PROTOCOL_VERSION.as_bytes());
        frame(&mut h, BINDING_PROFILE.as_bytes());
        frame(&mut h, LOCK_PATH.as_bytes());
        frame(&mut h, &self.broker_binding_digest);
        frame(&mut h, &self.lock_device.to_le_bytes());
        frame(&mut h, &self.lock_inode.to_le_bytes());
        frame(&mut h, self.binding_ref.as_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

/// Trusted broker-side entry point. This type does not expose a raw
/// `AdminWebsocket` and does not provide an unlocked coordinator update API.
pub struct TrustedAdminMutationExclusion {
    binding: AdminMutationExclusionBinding,
    binding_digest: [u8; 32],
}

impl TrustedAdminMutationExclusion {
    pub fn new(
        binding: AdminMutationExclusionBinding,
        broker_binding: &BrokerProcessBinding,
    ) -> Result<Self, AdminMutationExclusionError> {
        binding.validate()?;
        let broker_digest = broker_binding
            .binding_digest()
            .map_err(AdminMutationExclusionError::AdminIsolation)?;
        if broker_digest != binding.broker_binding_digest {
            return Err(AdminMutationExclusionError::BrokerBindingMismatch);
        }
        let binding_digest = binding.binding_digest()?;
        Ok(Self {
            binding,
            binding_digest,
        })
    }

    pub fn binding_digest(&self) -> [u8; 32] {
        self.binding_digest
    }

    /// Begin the effect-side critical section for one exact already-durable
    /// response attempt. The shared Admin mutation lock is acquired BEFORE #535
    /// isolation begins and remains owned until `finish()` returns.
    pub fn begin_response_critical_section<'a>(
        &self,
        conductor_store: &'a TrustedConductorProcessStore,
        broker_binding: &BrokerProcessBinding,
        reserved_attempt: &DurablyReservedResponseAttempt,
    ) -> Result<ResponseAdminMutationExclusionGuard<'a>, AdminMutationExclusionError> {
        self.require_broker_binding(broker_binding)?;
        if !reserved_attempt.durably_reserved_here() {
            return Err(AdminMutationExclusionError::AttemptNotDurable);
        }
        let reservation_digest = reserved_attempt.reservation_digest();
        if reservation_digest.0 == [0u8; 32] {
            return Err(AdminMutationExclusionError::AttemptNotDurable);
        }

        let lock = self.acquire_lock()?;
        let isolation = begin_exclusive_admin_isolation(conductor_store, broker_binding)
            .map_err(AdminMutationExclusionError::AdminIsolation)?;
        lock.validate(&self.binding)?;
        let started_at_ms = now_ms()?;

        Ok(ResponseAdminMutationExclusionGuard {
            lock,
            isolation,
            binding: self.binding.clone(),
            binding_digest: self.binding_digest,
            reservation_digest,
            effect_identity_digest: reserved_attempt.prepared().effect_identity_digest(),
            journal_ref: reserved_attempt.journal_ref().to_string(),
            started_at_ms,
        })
    }

    /// The only v0.1 broker operation permitted to replace coordinator code.
    ///
    /// It acquires the same exclusion inode as the response critical section,
    /// opens the same #535 isolated interval, performs the direct Holochain Admin
    /// mutation, closes the websocket, then closes #535 before releasing the lock.
    pub async fn update_coordinators_exclusive(
        &self,
        conductor_store: &TrustedConductorProcessStore,
        broker_binding: &BrokerProcessBinding,
        payload: UpdateCoordinatorsPayload,
    ) -> Result<QualifiedCoordinatorUpdateOperation, AdminMutationExclusionError> {
        self.require_broker_binding(broker_binding)?;
        let lock = self.acquire_lock()?;
        let isolation = begin_exclusive_admin_isolation(conductor_store, broker_binding)
            .map_err(AdminMutationExclusionError::AdminIsolation)?;
        lock.validate(&self.binding)?;

        let endpoint = isolation.admin_endpoint();
        let target_dna_hash_raw_39 = payload.cell_id.dna_hash().get_raw_39().to_vec();
        let target_agent_pub_key_raw_39 = payload.cell_id.agent_pubkey().get_raw_39().to_vec();

        let admin = AdminWebsocket::connect(endpoint, Some(BROKER_ORIGIN.to_string()))
            .await
            .map_err(|error| AdminMutationExclusionError::Conductor(error.to_string()))?;
        admin
            .update_coordinators(payload)
            .await
            .map_err(|error| AdminMutationExclusionError::Conductor(error.to_string()))?;
        drop(admin);

        let isolation = isolation
            .finish()
            .map_err(AdminMutationExclusionError::AdminIsolation)?;
        lock.validate(&self.binding)?;
        let completed_at_ms = now_ms()?;
        if completed_at_ms < isolation.ended_at_ms() {
            return Err(AdminMutationExclusionError::InvalidInterval);
        }

        let mutation_digest = coordinator_update_digest(
            self.binding_digest,
            &isolation,
            &target_dna_hash_raw_39,
            &target_agent_pub_key_raw_39,
            completed_at_ms,
        );

        Ok(QualifiedCoordinatorUpdateOperation {
            exclusion_binding_digest: self.binding_digest,
            target_dna_hash_raw_39,
            target_agent_pub_key_raw_39,
            isolation,
            mutation_digest,
            completed_at_ms,
        })
    }

    fn require_broker_binding(
        &self,
        broker_binding: &BrokerProcessBinding,
    ) -> Result<(), AdminMutationExclusionError> {
        let digest = broker_binding
            .binding_digest()
            .map_err(AdminMutationExclusionError::AdminIsolation)?;
        if digest != self.binding.broker_binding_digest {
            Err(AdminMutationExclusionError::BrokerBindingMismatch)
        } else {
            Ok(())
        }
    }

    fn acquire_lock(&self) -> Result<ExclusiveAdminMutationLock, AdminMutationExclusionError> {
        validate_lock_parent()?;
        let path = Path::new(LOCK_PATH);
        let mut options = OpenOptions::new();
        options
            .read(true)
            .write(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC);
        let file = options.open(path).map_err(AdminMutationExclusionError::Io)?;
        validate_lock_file(&file, &self.binding)?;

        let rc = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX) };
        if rc != 0 {
            return Err(AdminMutationExclusionError::Io(io::Error::last_os_error()));
        }
        validate_lock_file(&file, &self.binding)?;
        Ok(ExclusiveAdminMutationLock { file })
    }
}

/// Non-serializable guard proving the shared mutation lock and #535 isolation
/// interval are simultaneously open for one exact durable response attempt.
pub struct ResponseAdminMutationExclusionGuard<'a> {
    lock: ExclusiveAdminMutationLock,
    isolation: ExclusiveAdminIsolationGuard<'a>,
    binding: AdminMutationExclusionBinding,
    binding_digest: [u8; 32],
    reservation_digest: Digest32,
    effect_identity_digest: Digest32,
    journal_ref: String,
    started_at_ms: u64,
}

impl ResponseAdminMutationExclusionGuard<'_> {
    pub fn admin_endpoint(&self) -> std::net::SocketAddr {
        self.isolation.admin_endpoint()
    }

    pub fn reservation_digest(&self) -> Digest32 {
        self.reservation_digest
    }

    pub fn effect_identity_digest(&self) -> Digest32 {
        self.effect_identity_digest
    }

    /// Finish the caller-controlled response interval while the exact shared
    /// mutation lock is still held. Only after #535 closes and the lock inode is
    /// revalidated can historical coordinator-update exclusion be returned.
    pub fn finish(
        self,
    ) -> Result<QualifiedResponseAdminMutationExclusion, AdminMutationExclusionError> {
        let Self {
            lock,
            isolation,
            binding,
            binding_digest,
            reservation_digest,
            effect_identity_digest,
            journal_ref,
            started_at_ms,
        } = self;

        lock.validate(&binding)?;
        let isolation = isolation
            .finish()
            .map_err(AdminMutationExclusionError::AdminIsolation)?;
        lock.validate(&binding)?;
        let ended_at_ms = now_ms()?;
        if ended_at_ms < started_at_ms || isolation.ended_at_ms() > ended_at_ms {
            return Err(AdminMutationExclusionError::InvalidInterval);
        }

        let exclusion_digest = response_exclusion_digest(
            binding_digest,
            reservation_digest,
            effect_identity_digest,
            &journal_ref,
            &isolation,
            started_at_ms,
            ended_at_ms,
        );

        Ok(QualifiedResponseAdminMutationExclusion {
            exclusion_binding_digest: binding_digest,
            reservation_digest,
            effect_identity_digest,
            journal_ref,
            isolation,
            exclusion_digest,
            started_at_ms,
            ended_at_ms,
        })
    }
}

/// Historical proof that one exact durable response attempt occupied the same
/// exclusion domain required by coordinator update throughout its guarded
/// interval. It does not prove what arbitrary caller work occurred inside.
#[derive(Clone, Debug, Serialize)]
pub struct QualifiedResponseAdminMutationExclusion {
    exclusion_binding_digest: [u8; 32],
    reservation_digest: Digest32,
    effect_identity_digest: Digest32,
    journal_ref: String,
    isolation: QualifiedExclusiveAdminIsolation,
    exclusion_digest: [u8; 32],
    started_at_ms: u64,
    ended_at_ms: u64,
}

impl QualifiedResponseAdminMutationExclusion {
    pub fn exclusion_binding_digest(&self) -> [u8; 32] {
        self.exclusion_binding_digest
    }

    pub fn reservation_digest(&self) -> Digest32 {
        self.reservation_digest
    }

    pub fn effect_identity_digest(&self) -> Digest32 {
        self.effect_identity_digest
    }

    pub fn journal_ref(&self) -> &str {
        &self.journal_ref
    }

    pub fn isolation(&self) -> &QualifiedExclusiveAdminIsolation {
        &self.isolation
    }

    pub fn exclusion_digest(&self) -> [u8; 32] {
        self.exclusion_digest
    }

    pub fn exclusion_profile(&self) -> &str {
        RESPONSE_EXCLUSION_PROFILE
    }

    pub fn started_at_ms(&self) -> u64 {
        self.started_at_ms
    }

    pub fn ended_at_ms(&self) -> u64 {
        self.ended_at_ms
    }

    pub const fn durable_attempt_bound_here(&self) -> bool {
        true
    }

    pub const fn coordinator_update_excluded_here(&self) -> bool {
        true
    }

    pub const fn effect_started_here(&self) -> bool {
        false
    }

    pub const fn work_inside_interval_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Non-deserializable audit result for one coordinator update that acquired the
/// same exact exclusion domain as response critical sections.
#[derive(Clone, Debug, Serialize)]
pub struct QualifiedCoordinatorUpdateOperation {
    exclusion_binding_digest: [u8; 32],
    target_dna_hash_raw_39: Vec<u8>,
    target_agent_pub_key_raw_39: Vec<u8>,
    isolation: QualifiedExclusiveAdminIsolation,
    mutation_digest: [u8; 32],
    completed_at_ms: u64,
}

impl QualifiedCoordinatorUpdateOperation {
    pub fn exclusion_binding_digest(&self) -> [u8; 32] {
        self.exclusion_binding_digest
    }

    pub fn target_dna_hash_raw_39(&self) -> &[u8] {
        &self.target_dna_hash_raw_39
    }

    pub fn target_agent_pub_key_raw_39(&self) -> &[u8] {
        &self.target_agent_pub_key_raw_39
    }

    pub fn isolation(&self) -> &QualifiedExclusiveAdminIsolation {
        &self.isolation
    }

    pub fn mutation_digest(&self) -> [u8; 32] {
        self.mutation_digest
    }

    pub fn mutation_profile(&self) -> &str {
        COORDINATOR_UPDATE_PROFILE
    }

    pub fn completed_at_ms(&self) -> u64 {
        self.completed_at_ms
    }

    pub const fn shared_admin_mutation_exclusion_held_here(&self) -> bool {
        true
    }

    pub const fn coordinator_update_serialized_here(&self) -> bool {
        true
    }

    pub const fn resulting_deployment_approved_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

struct ExclusiveAdminMutationLock {
    file: File,
}

impl ExclusiveAdminMutationLock {
    fn validate(
        &self,
        binding: &AdminMutationExclusionBinding,
    ) -> Result<(), AdminMutationExclusionError> {
        validate_lock_file(&self.file, binding)
    }
}

fn validate_lock_parent() -> Result<(), AdminMutationExclusionError> {
    let path = Path::new(LOCK_PATH);
    let parent = path
        .parent()
        .ok_or(AdminMutationExclusionError::InvalidLockPath)?;
    if !parent.is_absolute() {
        return Err(AdminMutationExclusionError::InvalidLockPath);
    }
    let canonical = fs::canonicalize(parent).map_err(AdminMutationExclusionError::Io)?;
    if canonical != PathBuf::from(parent) {
        return Err(AdminMutationExclusionError::NonCanonicalLockDirectory);
    }
    let metadata = fs::symlink_metadata(parent).map_err(AdminMutationExclusionError::Io)?;
    if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
        return Err(AdminMutationExclusionError::UnsafeLockDirectory);
    }
    let euid = unsafe { libc::geteuid() };
    if metadata.uid() != euid {
        return Err(AdminMutationExclusionError::LockOwnerMismatch);
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != LOCK_DIR_MODE {
        return Err(AdminMutationExclusionError::UnsafeLockDirectoryMode {
            expected: LOCK_DIR_MODE,
            observed: mode,
        });
    }
    Ok(())
}

fn validate_lock_file(
    file: &File,
    binding: &AdminMutationExclusionBinding,
) -> Result<(), AdminMutationExclusionError> {
    let metadata = file.metadata().map_err(AdminMutationExclusionError::Io)?;
    if !metadata.file_type().is_file() {
        return Err(AdminMutationExclusionError::UnsafeLockFile);
    }
    let euid = unsafe { libc::geteuid() };
    if metadata.uid() != euid {
        return Err(AdminMutationExclusionError::LockOwnerMismatch);
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != LOCK_FILE_MODE {
        return Err(AdminMutationExclusionError::UnsafeLockFileMode {
            expected: LOCK_FILE_MODE,
            observed: mode,
        });
    }
    if metadata.dev() != binding.lock_device || metadata.ino() != binding.lock_inode {
        return Err(AdminMutationExclusionError::LockIdentityMismatch);
    }
    Ok(())
}

fn response_exclusion_digest(
    binding_digest: [u8; 32],
    reservation_digest: Digest32,
    effect_identity_digest: Digest32,
    journal_ref: &str,
    isolation: &QualifiedExclusiveAdminIsolation,
    started_at_ms: u64,
    ended_at_ms: u64,
) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_RESPONSE);
    frame(&mut h, PROTOCOL_VERSION.as_bytes());
    frame(&mut h, RESPONSE_EXCLUSION_PROFILE.as_bytes());
    frame(&mut h, &binding_digest);
    frame(&mut h, &reservation_digest.0);
    frame(&mut h, &effect_identity_digest.0);
    frame(&mut h, journal_ref.as_bytes());
    frame(&mut h, &isolation.isolation_digest());
    frame(&mut h, isolation.isolation_profile().as_bytes());
    frame(&mut h, &started_at_ms.to_le_bytes());
    frame(&mut h, &ended_at_ms.to_le_bytes());
    *h.finalize().as_bytes()
}

fn coordinator_update_digest(
    binding_digest: [u8; 32],
    isolation: &QualifiedExclusiveAdminIsolation,
    dna_hash_raw_39: &[u8],
    agent_pub_key_raw_39: &[u8],
    completed_at_ms: u64,
) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_UPDATE);
    frame(&mut h, PROTOCOL_VERSION.as_bytes());
    frame(&mut h, COORDINATOR_UPDATE_PROFILE.as_bytes());
    frame(&mut h, &binding_digest);
    frame(&mut h, &isolation.isolation_digest());
    frame(&mut h, isolation.isolation_profile().as_bytes());
    frame(&mut h, dna_hash_raw_39);
    frame(&mut h, agent_pub_key_raw_39);
    frame(&mut h, &completed_at_ms.to_le_bytes());
    *h.finalize().as_bytes()
}

fn now_ms() -> Result<u64, AdminMutationExclusionError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| AdminMutationExclusionError::ClockBeforeUnixEpoch)?;
    let value = u64::try_from(duration.as_millis())
        .map_err(|_| AdminMutationExclusionError::ClockOverflow)?;
    if value == 0 {
        Err(AdminMutationExclusionError::ClockBeforeUnixEpoch)
    } else {
        Ok(value)
    }
}

fn validate_digest(
    value: &[u8; 32],
    field: &'static str,
) -> Result<(), AdminMutationExclusionError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(AdminMutationExclusionError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), AdminMutationExclusionError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(AdminMutationExclusionError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Debug)]
pub enum AdminMutationExclusionError {
    WrongProtocol,
    InvalidDigest(&'static str),
    InvalidText(&'static str),
    InvalidLockIdentity,
    InvalidLockPath,
    NonCanonicalLockDirectory,
    UnsafeLockDirectory,
    UnsafeLockDirectoryMode { expected: u32, observed: u32 },
    UnsafeLockFile,
    UnsafeLockFileMode { expected: u32, observed: u32 },
    LockOwnerMismatch,
    LockIdentityMismatch,
    BrokerBindingMismatch,
    AttemptNotDurable,
    InvalidInterval,
    ClockBeforeUnixEpoch,
    ClockOverflow,
    Conductor(String),
    Io(io::Error),
    AdminIsolation(AdminIsolationError),
}

impl fmt::Display for AdminMutationExclusionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong Admin mutation exclusion protocol"),
            Self::InvalidDigest(field) => write!(f, "invalid {field}"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::InvalidLockIdentity => write!(f, "invalid provisioned Admin mutation lock identity"),
            Self::InvalidLockPath => write!(f, "invalid fixed Admin mutation lock path"),
            Self::NonCanonicalLockDirectory => write!(f, "Admin mutation lock directory path is not canonical"),
            Self::UnsafeLockDirectory => write!(f, "Admin mutation lock parent is not a safe directory"),
            Self::UnsafeLockDirectoryMode { expected, observed } => write!(
                f,
                "unsafe Admin mutation lock directory mode: expected {expected:o}, observed {observed:o}"
            ),
            Self::UnsafeLockFile => write!(f, "Admin mutation lock is not a regular file"),
            Self::UnsafeLockFileMode { expected, observed } => write!(
                f,
                "unsafe Admin mutation lock file mode: expected {expected:o}, observed {observed:o}"
            ),
            Self::LockOwnerMismatch => write!(f, "Admin mutation lock ownership mismatch"),
            Self::LockIdentityMismatch => write!(f, "Admin mutation lock device/inode differs from provisioned binding"),
            Self::BrokerBindingMismatch => write!(f, "Admin mutation exclusion does not match the pinned broker binding"),
            Self::AttemptNotDurable => write!(f, "response attempt is not durably reserved"),
            Self::InvalidInterval => write!(f, "invalid Admin mutation exclusion interval"),
            Self::ClockBeforeUnixEpoch => write!(f, "system clock is before Unix epoch"),
            Self::ClockOverflow => write!(f, "system clock does not fit u64 milliseconds"),
            Self::Conductor(error) => write!(f, "Holochain Admin mutation failed: {error}"),
            Self::Io(error) => write!(f, "Admin mutation exclusion filesystem error: {error}"),
            Self::AdminIsolation(error) => write!(f, "Admin isolation failed: {error}"),
        }
    }
}

impl std::error::Error for AdminMutationExclusionError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn binding() -> AdminMutationExclusionBinding {
        AdminMutationExclusionBinding {
            protocol_version: PROTOCOL_VERSION.into(),
            broker_binding_digest: d(1),
            lock_device: 100,
            lock_inode: 200,
            binding_ref: "provisioning:admin-mutation-lock:1".into(),
        }
    }

    #[test]
    fn exclusion_binding_is_stable() {
        let a = binding();
        let b = a.clone();
        assert_eq!(a.binding_digest().unwrap(), b.binding_digest().unwrap());
    }

    #[test]
    fn lock_inode_changes_binding_identity() {
        let a = binding();
        let mut b = a.clone();
        b.lock_inode += 1;
        assert_ne!(a.binding_digest().unwrap(), b.binding_digest().unwrap());
    }

    #[test]
    fn broker_binding_changes_exclusion_identity() {
        let a = binding();
        let mut b = a.clone();
        b.broker_binding_digest = d(9);
        assert_ne!(a.binding_digest().unwrap(), b.binding_digest().unwrap());
    }

    #[test]
    fn zero_lock_identity_denies() {
        let mut value = binding();
        value.lock_inode = 0;
        assert!(matches!(
            value.validate().unwrap_err(),
            AdminMutationExclusionError::InvalidLockIdentity
        ));
    }
}

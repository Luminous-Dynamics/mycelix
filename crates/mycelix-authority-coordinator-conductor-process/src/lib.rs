// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Linux host process/listener stability fence for one pinned Holochain Admin endpoint.
//!
//! This crate closes a narrower gap than cryptographic conductor identity. It pins an
//! exact loopback endpoint, process UID and executable identity out of band; resolves
//! the endpoint's unique listening socket owner through Linux procfs; opens a pidfd;
//! and requires the same listener inode, PID/start-time and executable identity before
//! and after a caller-controlled observation interval.
//!
//! The resulting fence is historical interval evidence only. It has no future lease
//! and does not prove that arbitrary work performed between `begin_fence` and `finish`
//! actually used this endpoint. A later native admission orchestrator must bind its
//! target/code observation timestamps and endpoint to this exact fence.
//!
//! Same-UID process tampering, root compromise, transient exec-and-restore ABA inside
//! the interval, kernel/procfs compromise and full-machine rollback remain outside the
//! v0.1 theorem.

#[cfg(not(target_os = "linux"))]
compile_error!("coordinator conductor process fence v0.1 requires Linux");

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-conductor-process-v0.1";
pub const BINDING_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-process-binding-v1-blake3-framed";
pub const BINDING_PIN_PROTOCOL: &str =
    "mycelix-authority-coordinator-conductor-process-binding-pin-v0.1";
pub const BINDING_PIN_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-process-binding-pin-v1-blake3-framed";
pub const EXECUTABLE_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-executable-v1-blake3-bytes";
pub const PROCESS_SNAPSHOT_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-process-snapshot-v1-blake3-framed";
pub const STATE_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-process-state-v1-blake3-framed";
pub const FENCE_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-process-fence-v1-blake3-framed";
pub const STATE_FILE_MODE: u32 = 0o600;
pub const MAX_STATE_BYTES: usize = 1024 * 1024;
pub const MAX_EXECUTABLE_BYTES: u64 = 1024 * 1024 * 1024;

const DOMAIN_BINDING: &[u8] = b"mycelix/authority/coordinator-conductor-process-binding/v1";
const DOMAIN_BINDING_PIN: &[u8] =
    b"mycelix/authority/coordinator-conductor-process-binding-pin/v1";
const DOMAIN_SNAPSHOT: &[u8] = b"mycelix/authority/coordinator-conductor-process-snapshot/v1";
const DOMAIN_STATE: &[u8] = b"mycelix/authority/coordinator-conductor-process-state/v1";
const DOMAIN_FENCE: &[u8] = b"mycelix/authority/coordinator-conductor-process-fence/v1";
const MAX_TEXT_BYTES: usize = 4096;
const TCP_LISTEN_STATE: &str = "0A";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConductorProcessBinding {
    pub protocol_version: String,
    pub binding_id: String,
    pub admin_endpoint: SocketAddr,
    pub expected_process_uid: u32,
    pub executable_path: String,
    pub executable_digest: [u8; 32],
    pub executable_profile: String,
    pub binding_ref: String,
}

impl ConductorProcessBinding {
    pub fn validate(&self) -> Result<(), ConductorProcessError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(ConductorProcessError::WrongProtocol);
        }
        validate_text(&self.binding_id, "conductor process binding id")?;
        if self.admin_endpoint.port() == 0 || !self.admin_endpoint.ip().is_loopback() {
            return Err(ConductorProcessError::InvalidAdminEndpoint);
        }
        validate_text(&self.executable_path, "conductor executable path")?;
        if !Path::new(&self.executable_path).is_absolute() {
            return Err(ConductorProcessError::ExecutablePathNotAbsolute);
        }
        validate_digest(&self.executable_digest, "conductor executable digest")?;
        if self.executable_profile != EXECUTABLE_PROFILE {
            return Err(ConductorProcessError::WrongExecutableProfile);
        }
        validate_text(&self.binding_ref, "conductor process binding ref")?;
        Ok(())
    }

    pub fn binding_digest(&self) -> Result<[u8; 32], ConductorProcessError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_BINDING);
        frame(&mut h, PROTOCOL_VERSION.as_bytes());
        frame(&mut h, BINDING_PROFILE.as_bytes());
        frame(&mut h, self.binding_id.as_bytes());
        frame(&mut h, self.admin_endpoint.to_string().as_bytes());
        frame(&mut h, &self.expected_process_uid.to_le_bytes());
        frame(&mut h, self.executable_path.as_bytes());
        frame(&mut h, &self.executable_digest);
        frame(&mut h, self.executable_profile.as_bytes());
        frame(&mut h, self.binding_ref.as_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConductorProcessBindingPin {
    pub pin_protocol_version: String,
    pub pin_profile: String,
    pub binding_digest: [u8; 32],
    pub binding_profile: String,
}

impl ConductorProcessBindingPin {
    pub fn for_binding(binding: &ConductorProcessBinding) -> Result<Self, ConductorProcessError> {
        Ok(Self {
            pin_protocol_version: BINDING_PIN_PROTOCOL.into(),
            pin_profile: BINDING_PIN_PROFILE.into(),
            binding_digest: binding.binding_digest()?,
            binding_profile: BINDING_PROFILE.into(),
        })
    }

    pub fn validate(&self) -> Result<(), ConductorProcessError> {
        if self.pin_protocol_version != BINDING_PIN_PROTOCOL {
            return Err(ConductorProcessError::WrongBindingPinProtocol);
        }
        if self.pin_profile != BINDING_PIN_PROFILE {
            return Err(ConductorProcessError::WrongBindingPinProfile);
        }
        validate_digest(&self.binding_digest, "conductor binding pin digest")?;
        if self.binding_profile != BINDING_PROFILE {
            return Err(ConductorProcessError::WrongBindingProfile);
        }
        Ok(())
    }

    pub fn pin_digest(&self) -> Result<[u8; 32], ConductorProcessError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_BINDING_PIN);
        frame(&mut h, BINDING_PIN_PROTOCOL.as_bytes());
        frame(&mut h, BINDING_PIN_PROFILE.as_bytes());
        frame(&mut h, &self.binding_digest);
        frame(&mut h, self.binding_profile.as_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConductorProcessSnapshot {
    pub listener_inode: u64,
    pub pid: u32,
    pub process_start_time_ticks: u64,
    pub effective_uid: u32,
    pub executable_path: String,
    pub executable_digest: [u8; 32],
    pub executable_dev: u64,
    pub executable_inode: u64,
    pub executable_len: u64,
}

impl ConductorProcessSnapshot {
    pub fn validate_against(
        &self,
        binding: &ConductorProcessBinding,
    ) -> Result<(), ConductorProcessError> {
        if self.listener_inode == 0 || self.pid == 0 || self.process_start_time_ticks == 0 {
            return Err(ConductorProcessError::InvalidProcessSnapshot);
        }
        if self.effective_uid != binding.expected_process_uid {
            return Err(ConductorProcessError::ProcessUidMismatch {
                expected: binding.expected_process_uid,
                observed: self.effective_uid,
            });
        }
        if self.executable_path != binding.executable_path {
            return Err(ConductorProcessError::ExecutablePathMismatch);
        }
        if self.executable_digest != binding.executable_digest {
            return Err(ConductorProcessError::ExecutableDigestMismatch);
        }
        if self.executable_dev == 0 || self.executable_inode == 0 || self.executable_len == 0 {
            return Err(ConductorProcessError::InvalidExecutableMetadata);
        }
        Ok(())
    }

    pub fn snapshot_digest(&self) -> Result<[u8; 32], ConductorProcessError> {
        validate_digest(&self.executable_digest, "process executable digest")?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_SNAPSHOT);
        frame(&mut h, PROCESS_SNAPSHOT_PROFILE.as_bytes());
        frame(&mut h, &self.listener_inode.to_le_bytes());
        frame(&mut h, &self.pid.to_le_bytes());
        frame(&mut h, &self.process_start_time_ticks.to_le_bytes());
        frame(&mut h, &self.effective_uid.to_le_bytes());
        frame(&mut h, self.executable_path.as_bytes());
        frame(&mut h, &self.executable_digest);
        frame(&mut h, &self.executable_dev.to_le_bytes());
        frame(&mut h, &self.executable_inode.to_le_bytes());
        frame(&mut h, &self.executable_len.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

/// Serialized local state is continuity data, not positive process authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedConductorProcessState {
    pub protocol_version: String,
    pub state_profile: String,
    pub state_generation: u64,
    pub previous_state_digest: Option<[u8; 32]>,
    pub binding: ConductorProcessBinding,
    pub binding_digest: [u8; 32],
    pub binding_pin_digest: [u8; 32],
    pub last_fence_snapshot_digest: Option<[u8; 32]>,
    pub last_trusted_time_ms: u64,
    pub state_digest: [u8; 32],
}

impl TrustedConductorProcessState {
    fn validate(&self) -> Result<(), ConductorProcessError> {
        if self.protocol_version != PROTOCOL_VERSION || self.state_profile != STATE_PROFILE {
            return Err(ConductorProcessError::WrongStateProtocol);
        }
        if self.state_generation == 0 {
            return Err(ConductorProcessError::InvalidStateGeneration);
        }
        match (self.state_generation, self.previous_state_digest) {
            (1, None) => {}
            (1, Some(_)) => return Err(ConductorProcessError::InvalidPreviousStateDigest),
            (_, Some(previous)) => validate_digest(&previous, "previous conductor state digest")?,
            (_, None) => return Err(ConductorProcessError::InvalidPreviousStateDigest),
        }
        self.binding.validate()?;
        if self.binding.binding_digest()? != self.binding_digest {
            return Err(ConductorProcessError::BindingDigestMismatch);
        }
        validate_digest(&self.binding_pin_digest, "binding pin digest")?;
        if let Some(snapshot) = self.last_fence_snapshot_digest {
            validate_digest(&snapshot, "last conductor fence snapshot digest")?;
        }
        if self.last_trusted_time_ms == 0 {
            return Err(ConductorProcessError::InvalidTrustedClockFloor);
        }
        if self.compute_digest()? != self.state_digest {
            return Err(ConductorProcessError::StateDigestMismatch);
        }
        Ok(())
    }

    fn compute_digest(&self) -> Result<[u8; 32], ConductorProcessError> {
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_STATE);
        frame(&mut h, PROTOCOL_VERSION.as_bytes());
        frame(&mut h, STATE_PROFILE.as_bytes());
        frame(&mut h, &self.state_generation.to_le_bytes());
        match self.previous_state_digest {
            Some(previous) => {
                frame(&mut h, b"previous");
                frame(&mut h, &previous);
            }
            None => frame(&mut h, b"genesis"),
        }
        frame(&mut h, &self.binding_digest);
        frame(&mut h, &self.binding_pin_digest);
        match self.last_fence_snapshot_digest {
            Some(snapshot) => {
                frame(&mut h, b"snapshot");
                frame(&mut h, &snapshot);
            }
            None => frame(&mut h, b"no-snapshot"),
        }
        frame(&mut h, &self.last_trusted_time_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }

    fn advance_fence(
        &self,
        snapshot_digest: [u8; 32],
        trusted_time_ms: u64,
    ) -> Result<Self, ConductorProcessError> {
        if trusted_time_ms < self.last_trusted_time_ms {
            return Err(ConductorProcessError::ClockRollback {
                stored_ms: self.last_trusted_time_ms,
                observed_ms: trusted_time_ms,
            });
        }
        validate_digest(&snapshot_digest, "completed conductor snapshot digest")?;
        let state_generation = self
            .state_generation
            .checked_add(1)
            .ok_or(ConductorProcessError::StateGenerationOverflow)?;
        let mut next = Self {
            protocol_version: PROTOCOL_VERSION.into(),
            state_profile: STATE_PROFILE.into(),
            state_generation,
            previous_state_digest: Some(self.state_digest),
            binding: self.binding.clone(),
            binding_digest: self.binding_digest,
            binding_pin_digest: self.binding_pin_digest,
            last_fence_snapshot_digest: Some(snapshot_digest),
            last_trusted_time_ms: trusted_time_ms,
            state_digest: [0; 32],
        };
        next.state_digest = next.compute_digest()?;
        next.validate()?;
        Ok(next)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConductorProcessStateSummary {
    pub state_generation: u64,
    pub state_digest: [u8; 32],
    pub binding_digest: [u8; 32],
    pub admin_endpoint: SocketAddr,
    pub expected_process_uid: u32,
    pub executable_path: String,
    pub executable_digest: [u8; 32],
    pub last_fence_snapshot_digest: Option<[u8; 32]>,
    pub last_trusted_time_ms: u64,
}

impl From<&TrustedConductorProcessState> for ConductorProcessStateSummary {
    fn from(state: &TrustedConductorProcessState) -> Self {
        Self {
            state_generation: state.state_generation,
            state_digest: state.state_digest,
            binding_digest: state.binding_digest,
            admin_endpoint: state.binding.admin_endpoint,
            expected_process_uid: state.binding.expected_process_uid,
            executable_path: state.binding.executable_path.clone(),
            executable_digest: state.binding.executable_digest,
            last_fence_snapshot_digest: state.last_fence_snapshot_digest,
            last_trusted_time_ms: state.last_trusted_time_ms,
        }
    }
}

/// Historical non-deserializable process/listener stability evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedConductorProcessFence {
    admin_endpoint: SocketAddr,
    binding_digest: [u8; 32],
    binding_profile: String,
    process_snapshot: ConductorProcessSnapshot,
    process_snapshot_digest: [u8; 32],
    process_snapshot_profile: String,
    persisted_state_digest: [u8; 32],
    persisted_state_profile: String,
    qualification_digest: [u8; 32],
    qualification_profile: String,
    started_at_ms: u64,
    ended_at_ms: u64,
}

impl QualifiedConductorProcessFence {
    pub fn admin_endpoint(&self) -> SocketAddr {
        self.admin_endpoint
    }
    pub fn binding_digest(&self) -> [u8; 32] {
        self.binding_digest
    }
    pub fn process_snapshot(&self) -> &ConductorProcessSnapshot {
        &self.process_snapshot
    }
    pub fn process_snapshot_digest(&self) -> [u8; 32] {
        self.process_snapshot_digest
    }
    pub fn persisted_state_digest(&self) -> [u8; 32] {
        self.persisted_state_digest
    }
    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }
    pub fn started_at_ms(&self) -> u64 {
        self.started_at_ms
    }
    pub fn ended_at_ms(&self) -> u64 {
        self.ended_at_ms
    }
    pub fn binding_profile(&self) -> &str {
        &self.binding_profile
    }
    pub fn process_snapshot_profile(&self) -> &str {
        &self.process_snapshot_profile
    }
    pub fn persisted_state_profile(&self) -> &str {
        &self.persisted_state_profile
    }
    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }
}

#[derive(Clone, Debug)]
pub struct TrustedConductorProcessStore {
    state_path: PathBuf,
    lock_path: PathBuf,
}

impl TrustedConductorProcessStore {
    pub fn new(state_path: impl Into<PathBuf>) -> Result<Self, ConductorProcessError> {
        let requested = state_path.into();
        let file_name = requested
            .file_name()
            .ok_or(ConductorProcessError::StatePathHasNoFileName)?
            .to_os_string();
        let requested_parent = requested
            .parent()
            .ok_or(ConductorProcessError::StatePathHasNoParent)?;
        let anchored_parent = if requested_parent.is_absolute() {
            requested_parent.to_path_buf()
        } else {
            std::env::current_dir()
                .map_err(io_error("resolve current directory"))?
                .join(requested_parent)
        };
        let canonical_parent = fs::canonicalize(&anchored_parent)
            .map_err(io_error("canonicalize conductor-state parent"))?;
        require_secure_parent_directory(&canonical_parent)?;
        let state_path = canonical_parent.join(&file_name);
        let lock_path = canonical_parent.join(format!("{}.lock", file_name.to_string_lossy()));
        Ok(Self {
            state_path,
            lock_path,
        })
    }

    /// First-use ceremony only. Normal process fencing accepts no binding or pin.
    pub fn bootstrap_from_out_of_band_pin(
        &self,
        binding: ConductorProcessBinding,
        pin: &ConductorProcessBindingPin,
    ) -> Result<ConductorProcessStateSummary, ConductorProcessError> {
        let _lock = self.acquire_exclusive_lock()?;
        self.require_state_absent()?;
        binding.validate()?;
        pin.validate()?;
        let binding_digest = binding.binding_digest()?;
        if pin.binding_digest != binding_digest || pin.binding_profile != BINDING_PROFILE {
            return Err(ConductorProcessError::BindingPinMismatch);
        }
        let pin_digest = pin.pin_digest()?;
        let now_ms = system_time_ms()?;
        let mut state = TrustedConductorProcessState {
            protocol_version: PROTOCOL_VERSION.into(),
            state_profile: STATE_PROFILE.into(),
            state_generation: 1,
            previous_state_digest: None,
            binding,
            binding_digest,
            binding_pin_digest: pin_digest,
            last_fence_snapshot_digest: None,
            last_trusted_time_ms: now_ms,
            state_digest: [0; 32],
        };
        state.state_digest = state.compute_digest()?;
        state.validate()?;
        self.atomic_replace_state(&state)?;
        Ok((&state).into())
    }

    pub fn state_summary(&self) -> Result<ConductorProcessStateSummary, ConductorProcessError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        Ok((&state).into())
    }

    /// Begin one historical process/listener stability interval.
    ///
    /// The guard retains the exclusive trusted-state lock and a pidfd until `finish`.
    /// Work that wishes to rely on this fence must be performed while the guard lives.
    pub fn begin_fence(&self) -> Result<ConductorProcessFenceGuard<'_>, ConductorProcessError> {
        let lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        let floor_checked_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;

        let first = resolve_listener_process(&state.binding)?;
        let pidfd = pidfd_open(first.pid)?;
        ensure_pidfd_alive(&pidfd)?;
        // Re-observe after obtaining the pidfd so numeric PID reuse between the first
        // procfs scan and pidfd acquisition cannot silently become positive evidence.
        let pre = resolve_listener_process(&state.binding)?;
        if first != pre {
            return Err(ConductorProcessError::ProcessChangedDuringFenceStart);
        }
        ensure_pidfd_alive(&pidfd)?;
        let started_at_ms = system_time_ms()?;
        if started_at_ms < floor_checked_at_ms || started_at_ms < state.last_trusted_time_ms {
            return Err(ConductorProcessError::ClockRollback {
                stored_ms: state.last_trusted_time_ms,
                observed_ms: started_at_ms,
            });
        }

        Ok(ConductorProcessFenceGuard {
            store: self,
            _lock: lock,
            state,
            pidfd,
            pre,
            started_at_ms,
        })
    }

    fn acquire_exclusive_lock(&self) -> Result<File, ConductorProcessError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(ConductorProcessError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.lock_path)?;
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(STATE_FILE_MODE)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.lock_path)
            .map_err(io_error("open conductor-state lock"))?;
        require_secure_regular_file(&lock, "conductor-state lock")?;
        lock.lock()
            .map_err(io_error("acquire conductor-state exclusive lock"))?;
        Ok(lock)
    }

    fn require_state_absent(&self) -> Result<(), ConductorProcessError> {
        match fs::symlink_metadata(&self.state_path) {
            Ok(_) => Err(ConductorProcessError::AlreadyInitialized),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(ConductorProcessError::Io {
                operation: "inspect conductor bootstrap state path",
                message: error.to_string(),
            }),
        }
    }

    fn load_state_unlocked(&self) -> Result<TrustedConductorProcessState, ConductorProcessError> {
        reject_symlink_if_present(&self.state_path)?;
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.state_path)
            .map_err(io_error("open conductor trusted state"))?;
        require_secure_regular_file(&file, "conductor trusted state")?;
        let len = file
            .metadata()
            .map_err(io_error("stat conductor trusted state"))?
            .len();
        if len == 0 || len > MAX_STATE_BYTES as u64 {
            return Err(ConductorProcessError::InvalidStateSize(len));
        }
        let mut bytes = Vec::with_capacity(len as usize);
        file.take((MAX_STATE_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(io_error("read conductor trusted state"))?;
        if bytes.len() > MAX_STATE_BYTES {
            return Err(ConductorProcessError::InvalidStateSize(bytes.len() as u64));
        }
        let state: TrustedConductorProcessState = serde_json::from_slice(&bytes)
            .map_err(|error| ConductorProcessError::Json(error.to_string()))?;
        state.validate()?;
        Ok(state)
    }

    fn atomic_replace_state(
        &self,
        state: &TrustedConductorProcessState,
    ) -> Result<(), ConductorProcessError> {
        state.validate()?;
        let parent = self
            .state_path
            .parent()
            .ok_or(ConductorProcessError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.state_path)?;
        let bytes = serde_json::to_vec(state)
            .map_err(|error| ConductorProcessError::Json(error.to_string()))?;
        if bytes.len() + 1 > MAX_STATE_BYTES {
            return Err(ConductorProcessError::InvalidStateSize((bytes.len() + 1) as u64));
        }
        let temp_path = self.unique_temp_path(state.state_generation)?;
        let result = (|| {
            let mut file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .mode(STATE_FILE_MODE)
                .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
                .open(&temp_path)
                .map_err(io_error("create conductor-state temp"))?;
            require_secure_regular_file(&file, "conductor-state temp")?;
            file.write_all(&bytes)
                .map_err(io_error("write conductor-state temp"))?;
            file.write_all(b"\n")
                .map_err(io_error("finish conductor-state temp"))?;
            file.sync_all()
                .map_err(io_error("fsync conductor-state temp"))?;
            drop(file);
            fs::rename(&temp_path, &self.state_path)
                .map_err(io_error("atomically replace conductor trusted state"))?;
            let directory = OpenOptions::new()
                .read(true)
                .custom_flags(libc::O_DIRECTORY | libc::O_CLOEXEC)
                .open(parent)
                .map_err(io_error("open conductor-state directory"))?;
            directory
                .sync_all()
                .map_err(io_error("fsync conductor-state directory"))?;
            Ok(())
        })();
        if result.is_err() {
            let _ = fs::remove_file(&temp_path);
        }
        result
    }

    fn unique_temp_path(&self, generation: u64) -> Result<PathBuf, ConductorProcessError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(ConductorProcessError::StatePathHasNoParent)?;
        let base = self
            .state_path
            .file_name()
            .ok_or(ConductorProcessError::StatePathHasNoFileName)?
            .to_string_lossy();
        let stamp = system_time_ms()?;
        for nonce in 0_u32..64 {
            let candidate = parent.join(format!(
                ".{base}.tmp.{}.{}.{}.{}",
                std::process::id(),
                generation,
                stamp,
                nonce
            ));
            if !candidate.exists() {
                return Ok(candidate);
            }
        }
        Err(ConductorProcessError::TemporaryPathExhausted)
    }
}

pub struct ConductorProcessFenceGuard<'a> {
    store: &'a TrustedConductorProcessStore,
    _lock: File,
    state: TrustedConductorProcessState,
    pidfd: OwnedFd,
    pre: ConductorProcessSnapshot,
    started_at_ms: u64,
}

impl ConductorProcessFenceGuard<'_> {
    pub fn admin_endpoint(&self) -> SocketAddr {
        self.state.binding.admin_endpoint
    }

    pub fn started_at_ms(&self) -> u64 {
        self.started_at_ms
    }

    pub fn pre_snapshot(&self) -> &ConductorProcessSnapshot {
        &self.pre
    }

    /// Close the interval and durably record it before returning positive fence
    /// evidence. The result is historical and intentionally has no `valid_until_ms`.
    pub fn finish(self) -> Result<QualifiedConductorProcessFence, ConductorProcessError> {
        ensure_pidfd_alive(&self.pidfd)?;
        let post = resolve_listener_process(&self.state.binding)?;
        if post != self.pre {
            return Err(ConductorProcessError::ProcessChangedDuringFence);
        }
        ensure_pidfd_alive(&self.pidfd)?;
        let ended_at_ms = system_time_ms()?;
        if ended_at_ms < self.started_at_ms || ended_at_ms < self.state.last_trusted_time_ms {
            return Err(ConductorProcessError::ClockRollback {
                stored_ms: self.state.last_trusted_time_ms.max(self.started_at_ms),
                observed_ms: ended_at_ms,
            });
        }

        let snapshot_digest = post.snapshot_digest()?;
        let next = self.state.advance_fence(snapshot_digest, ended_at_ms)?;
        self.store.atomic_replace_state(&next)?;

        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_FENCE);
        frame(&mut h, FENCE_PROFILE.as_bytes());
        frame(&mut h, &next.binding_digest);
        frame(&mut h, BINDING_PROFILE.as_bytes());
        frame(&mut h, &snapshot_digest);
        frame(&mut h, PROCESS_SNAPSHOT_PROFILE.as_bytes());
        frame(&mut h, &next.state_digest);
        frame(&mut h, STATE_PROFILE.as_bytes());
        frame(&mut h, &self.started_at_ms.to_le_bytes());
        frame(&mut h, &ended_at_ms.to_le_bytes());
        let qualification_digest = *h.finalize().as_bytes();

        Ok(QualifiedConductorProcessFence {
            admin_endpoint: next.binding.admin_endpoint,
            binding_digest: next.binding_digest,
            binding_profile: BINDING_PROFILE.into(),
            process_snapshot: post,
            process_snapshot_digest: snapshot_digest,
            process_snapshot_profile: PROCESS_SNAPSHOT_PROFILE.into(),
            persisted_state_digest: next.state_digest,
            persisted_state_profile: STATE_PROFILE.into(),
            qualification_digest,
            qualification_profile: FENCE_PROFILE.into(),
            started_at_ms: self.started_at_ms,
            ended_at_ms,
        })
    }
}

pub fn hash_executable_path(path: &Path) -> Result<[u8; 32], ConductorProcessError> {
    let mut file = File::open(path).map_err(io_error("open executable for hashing"))?;
    let metadata = file
        .metadata()
        .map_err(io_error("stat executable for hashing"))?;
    if !metadata.is_file() || metadata.len() == 0 || metadata.len() > MAX_EXECUTABLE_BYTES {
        return Err(ConductorProcessError::InvalidExecutableSize(metadata.len()));
    }
    if metadata.permissions().mode() & 0o022 != 0 {
        return Err(ConductorProcessError::ExecutableWritableByGroupOrOther);
    }
    let mut h = blake3::Hasher::new();
    let mut buffer = [0_u8; 1024 * 1024];
    loop {
        let read = file
            .read(&mut buffer)
            .map_err(io_error("hash executable bytes"))?;
        if read == 0 {
            break;
        }
        h.update(&buffer[..read]);
    }
    Ok(*h.finalize().as_bytes())
}

fn resolve_listener_process(
    binding: &ConductorProcessBinding,
) -> Result<ConductorProcessSnapshot, ConductorProcessError> {
    binding.validate()?;
    let listener_inode = resolve_unique_listener_inode(binding.admin_endpoint)?;
    let pid = resolve_unique_socket_owner_pid(listener_inode)?;
    let snapshot = snapshot_process(pid, listener_inode)?;
    snapshot.validate_against(binding)?;
    Ok(snapshot)
}

fn resolve_unique_listener_inode(endpoint: SocketAddr) -> Result<u64, ConductorProcessError> {
    let (path, expected_local) = match endpoint.ip() {
        IpAddr::V4(ip) => (Path::new("/proc/net/tcp"), proc_ipv4_local(ip, endpoint.port())),
        IpAddr::V6(ip) => (Path::new("/proc/net/tcp6"), proc_ipv6_local(ip, endpoint.port())),
    };
    let text = fs::read_to_string(path).map_err(io_error("read proc tcp listener table"))?;
    let mut inodes = BTreeSet::new();
    for line in text.lines().skip(1) {
        let fields: Vec<&str> = line.split_whitespace().collect();
        if fields.len() <= 9 || fields[1] != expected_local || fields[3] != TCP_LISTEN_STATE {
            continue;
        }
        let inode = fields[9]
            .parse::<u64>()
            .map_err(|_| ConductorProcessError::MalformedProcTcp)?;
        if inode != 0 {
            inodes.insert(inode);
        }
    }
    match inodes.len() {
        0 => Err(ConductorProcessError::ListenerNotFound(endpoint)),
        1 => Ok(*inodes.iter().next().expect("one inode")),
        _ => Err(ConductorProcessError::AmbiguousListener(endpoint)),
    }
}

fn proc_ipv4_local(ip: Ipv4Addr, port: u16) -> String {
    let encoded = if cfg!(target_endian = "little") {
        u32::from_le_bytes(ip.octets())
    } else {
        u32::from_be_bytes(ip.octets())
    };
    format!("{encoded:08X}:{port:04X}")
}

fn proc_ipv6_local(ip: Ipv6Addr, port: u16) -> String {
    let octets = ip.octets();
    let mut encoded = String::with_capacity(32);
    for chunk in octets.chunks_exact(4) {
        let bytes = [chunk[0], chunk[1], chunk[2], chunk[3]];
        let word = if cfg!(target_endian = "little") {
            u32::from_le_bytes(bytes)
        } else {
            u32::from_be_bytes(bytes)
        };
        encoded.push_str(&format!("{word:08X}"));
    }
    format!("{encoded}:{port:04X}")
}

fn resolve_unique_socket_owner_pid(inode: u64) -> Result<u32, ConductorProcessError> {
    let needle = format!("socket:[{inode}]");
    let mut owners = BTreeSet::new();
    for entry in fs::read_dir("/proc").map_err(io_error("list proc processes"))? {
        let entry = entry.map_err(io_error("read proc process entry"))?;
        let name = entry.file_name();
        let Some(name) = name.to_str() else {
            continue;
        };
        let Ok(pid) = name.parse::<u32>() else {
            continue;
        };
        let fd_dir = entry.path().join("fd");
        let Ok(fds) = fs::read_dir(fd_dir) else {
            continue;
        };
        let mut owned = false;
        for fd in fds.flatten() {
            if let Ok(target) = fs::read_link(fd.path()) {
                if target.to_string_lossy() == needle {
                    owned = true;
                    break;
                }
            }
        }
        if owned {
            owners.insert(pid);
        }
    }
    match owners.len() {
        0 => Err(ConductorProcessError::ListenerOwnerNotFound(inode)),
        1 => Ok(*owners.iter().next().expect("one process owner")),
        _ => Err(ConductorProcessError::AmbiguousListenerOwner(inode)),
    }
}

fn snapshot_process(
    pid: u32,
    listener_inode: u64,
) -> Result<ConductorProcessSnapshot, ConductorProcessError> {
    let process_start_time_ticks = read_process_start_time_ticks(pid)?;
    let effective_uid = read_process_effective_uid(pid)?;
    let exe_link = PathBuf::from(format!("/proc/{pid}/exe"));
    let executable_target = fs::read_link(&exe_link).map_err(io_error("read conductor proc exe"))?;
    let executable_path = executable_target.to_string_lossy().into_owned();
    if executable_path.ends_with(" (deleted)") || !Path::new(&executable_path).is_absolute() {
        return Err(ConductorProcessError::ExecutableDeletedOrNonAbsolute);
    }
    let executable = File::open(&exe_link).map_err(io_error("open conductor proc exe"))?;
    let metadata = executable
        .metadata()
        .map_err(io_error("stat conductor proc exe"))?;
    if !metadata.is_file() || metadata.len() == 0 || metadata.len() > MAX_EXECUTABLE_BYTES {
        return Err(ConductorProcessError::InvalidExecutableSize(metadata.len()));
    }
    if metadata.permissions().mode() & 0o022 != 0 {
        return Err(ConductorProcessError::ExecutableWritableByGroupOrOther);
    }
    let executable_digest = hash_open_file(executable)?;
    Ok(ConductorProcessSnapshot {
        listener_inode,
        pid,
        process_start_time_ticks,
        effective_uid,
        executable_path,
        executable_digest,
        executable_dev: metadata.dev(),
        executable_inode: metadata.ino(),
        executable_len: metadata.len(),
    })
}

fn hash_open_file(mut file: File) -> Result<[u8; 32], ConductorProcessError> {
    let mut h = blake3::Hasher::new();
    let mut buffer = [0_u8; 1024 * 1024];
    loop {
        let read = file
            .read(&mut buffer)
            .map_err(io_error("hash open conductor executable"))?;
        if read == 0 {
            break;
        }
        h.update(&buffer[..read]);
    }
    Ok(*h.finalize().as_bytes())
}

fn read_process_start_time_ticks(pid: u32) -> Result<u64, ConductorProcessError> {
    let text = fs::read_to_string(format!("/proc/{pid}/stat"))
        .map_err(io_error("read conductor process stat"))?;
    let close = text
        .rfind(')')
        .ok_or(ConductorProcessError::MalformedProcStat)?;
    let remainder = text
        .get(close + 1..)
        .ok_or(ConductorProcessError::MalformedProcStat)?
        .trim();
    // Remainder begins at field 3 (`state`); starttime is field 22 => index 19.
    let fields: Vec<&str> = remainder.split_whitespace().collect();
    fields
        .get(19)
        .ok_or(ConductorProcessError::MalformedProcStat)?
        .parse::<u64>()
        .map_err(|_| ConductorProcessError::MalformedProcStat)
}

fn read_process_effective_uid(pid: u32) -> Result<u32, ConductorProcessError> {
    let text = fs::read_to_string(format!("/proc/{pid}/status"))
        .map_err(io_error("read conductor process status"))?;
    let line = text
        .lines()
        .find(|line| line.starts_with("Uid:"))
        .ok_or(ConductorProcessError::MalformedProcStatus)?;
    let values: Vec<&str> = line.split_whitespace().collect();
    values
        .get(2)
        .ok_or(ConductorProcessError::MalformedProcStatus)?
        .parse::<u32>()
        .map_err(|_| ConductorProcessError::MalformedProcStatus)
}

fn pidfd_open(pid: u32) -> Result<OwnedFd, ConductorProcessError> {
    // SAFETY: pidfd_open takes a scalar PID and flags=0 and returns a new owned fd.
    let raw = unsafe { libc::syscall(libc::SYS_pidfd_open, pid as libc::pid_t, 0_u32) };
    if raw < 0 {
        return Err(ConductorProcessError::PidfdOpen {
            pid,
            message: std::io::Error::last_os_error().to_string(),
        });
    }
    // SAFETY: successful pidfd_open returns a unique owned file descriptor.
    Ok(unsafe { OwnedFd::from_raw_fd(raw as i32) })
}

fn ensure_pidfd_alive(pidfd: &OwnedFd) -> Result<(), ConductorProcessError> {
    let mut pollfd = libc::pollfd {
        fd: pidfd.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    // SAFETY: `pollfd` points to one valid pollfd and timeout 0 is nonblocking.
    let result = unsafe { libc::poll(&mut pollfd, 1, 0) };
    if result < 0 {
        return Err(ConductorProcessError::PidfdPoll(
            std::io::Error::last_os_error().to_string(),
        ));
    }
    if result > 0 && pollfd.revents & (libc::POLLIN | libc::POLLHUP | libc::POLLERR) != 0 {
        return Err(ConductorProcessError::ProcessExitedDuringFence);
    }
    Ok(())
}

fn checked_time_after_floor(floor_ms: u64) -> Result<u64, ConductorProcessError> {
    let now_ms = system_time_ms()?;
    if now_ms < floor_ms {
        Err(ConductorProcessError::ClockRollback {
            stored_ms: floor_ms,
            observed_ms: now_ms,
        })
    } else {
        Ok(now_ms)
    }
}

fn system_time_ms() -> Result<u64, ConductorProcessError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| ConductorProcessError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| ConductorProcessError::ClockOverflow)
}

fn reject_symlink_if_present(path: &Path) -> Result<(), ConductorProcessError> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            Err(ConductorProcessError::SymlinkNotAllowed(path.to_path_buf()))
        }
        Ok(_) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(ConductorProcessError::Io {
            operation: "inspect conductor-state path",
            message: error.to_string(),
        }),
    }
}

fn require_secure_parent_directory(path: &Path) -> Result<(), ConductorProcessError> {
    let metadata = fs::symlink_metadata(path).map_err(io_error("stat conductor-state directory"))?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(ConductorProcessError::InsecureParentDirectory);
    }
    if metadata.uid() != effective_uid() {
        return Err(ConductorProcessError::WrongOwner("conductor-state directory"));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode & 0o077 != 0 || mode & 0o200 == 0 {
        return Err(ConductorProcessError::InsecureDirectoryPermissions(mode));
    }
    Ok(())
}

fn require_secure_regular_file(
    file: &File,
    label: &'static str,
) -> Result<(), ConductorProcessError> {
    let metadata = file.metadata().map_err(io_error("stat conductor-state file"))?;
    if !metadata.is_file() {
        return Err(ConductorProcessError::NotRegularFile(label));
    }
    if metadata.uid() != effective_uid() {
        return Err(ConductorProcessError::WrongOwner(label));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != STATE_FILE_MODE {
        return Err(ConductorProcessError::InsecureFilePermissions { label, mode });
    }
    Ok(())
}

fn effective_uid() -> u32 {
    // SAFETY: geteuid takes no arguments and reads process credentials only.
    unsafe { libc::geteuid() }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), ConductorProcessError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(ConductorProcessError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn validate_digest(value: &[u8; 32], field: &'static str) -> Result<(), ConductorProcessError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(ConductorProcessError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

fn io_error(operation: &'static str) -> impl FnOnce(std::io::Error) -> ConductorProcessError {
    move |error| ConductorProcessError::Io {
        operation,
        message: error.to_string(),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConductorProcessError {
    WrongProtocol,
    WrongBindingProfile,
    WrongBindingPinProtocol,
    WrongBindingPinProfile,
    WrongExecutableProfile,
    InvalidAdminEndpoint,
    InvalidText(&'static str),
    InvalidDigest(&'static str),
    ExecutablePathNotAbsolute,
    BindingPinMismatch,
    BindingDigestMismatch,
    InvalidStateGeneration,
    StateGenerationOverflow,
    InvalidPreviousStateDigest,
    InvalidTrustedClockFloor,
    StateDigestMismatch,
    StatePathHasNoParent,
    StatePathHasNoFileName,
    AlreadyInitialized,
    InvalidStateSize(u64),
    TemporaryPathExhausted,
    SymlinkNotAllowed(PathBuf),
    InsecureParentDirectory,
    InsecureDirectoryPermissions(u32),
    InsecureFilePermissions { label: &'static str, mode: u32 },
    NotRegularFile(&'static str),
    WrongOwner(&'static str),
    ClockBeforeUnixEpoch,
    ClockOverflow,
    ClockRollback { stored_ms: u64, observed_ms: u64 },
    ListenerNotFound(SocketAddr),
    AmbiguousListener(SocketAddr),
    MalformedProcTcp,
    ListenerOwnerNotFound(u64),
    AmbiguousListenerOwner(u64),
    MalformedProcStat,
    MalformedProcStatus,
    InvalidProcessSnapshot,
    ProcessUidMismatch { expected: u32, observed: u32 },
    ExecutablePathMismatch,
    ExecutableDigestMismatch,
    ExecutableDeletedOrNonAbsolute,
    InvalidExecutableMetadata,
    InvalidExecutableSize(u64),
    ExecutableWritableByGroupOrOther,
    PidfdOpen { pid: u32, message: String },
    PidfdPoll(String),
    ProcessExitedDuringFence,
    ProcessChangedDuringFenceStart,
    ProcessChangedDuringFence,
    Io { operation: &'static str, message: String },
    Json(String),
}

impl fmt::Display for ConductorProcessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for ConductorProcessError {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::sync::atomic::{AtomicU64, Ordering};

    static COUNTER: AtomicU64 = AtomicU64::new(1);

    fn temp_dir() -> PathBuf {
        let id = COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "mycelix-conductor-process-{}-{id}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir(&path).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o700)).unwrap();
        path
    }

    fn self_executable() -> (String, [u8; 32]) {
        let target = fs::read_link("/proc/self/exe").unwrap();
        let path = target.to_string_lossy().into_owned();
        let digest = hash_executable_path(Path::new(&path)).unwrap();
        (path, digest)
    }

    fn binding(endpoint: SocketAddr) -> ConductorProcessBinding {
        let (path, digest) = self_executable();
        ConductorProcessBinding {
            protocol_version: PROTOCOL_VERSION.into(),
            binding_id: "test-conductor".into(),
            admin_endpoint: endpoint,
            expected_process_uid: effective_uid(),
            executable_path: path,
            executable_digest: digest,
            executable_profile: EXECUTABLE_PROFILE.into(),
            binding_ref: "test:current-process".into(),
        }
    }

    #[test]
    fn pin_has_independent_exact_wire_contract() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let candidate = binding(listener.local_addr().unwrap());
        let mut pin = ConductorProcessBindingPin::for_binding(&candidate).unwrap();
        assert!(pin.validate().is_ok());
        pin.pin_profile = "alternate".into();
        assert_eq!(
            pin.validate().unwrap_err(),
            ConductorProcessError::WrongBindingPinProfile
        );
    }

    #[test]
    fn proc_listener_resolution_binds_current_pid_start_and_executable() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let candidate = binding(listener.local_addr().unwrap());
        let snapshot = resolve_listener_process(&candidate).unwrap();
        assert_eq!(snapshot.pid, std::process::id());
        assert_eq!(snapshot.effective_uid, effective_uid());
        assert_eq!(snapshot.executable_path, candidate.executable_path);
        assert_eq!(snapshot.executable_digest, candidate.executable_digest);
        assert!(snapshot.listener_inode > 0);
        assert!(snapshot.process_start_time_ticks > 0);
    }

    #[test]
    fn wrong_executable_digest_denies_listener_owner() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut candidate = binding(listener.local_addr().unwrap());
        candidate.executable_digest = [99; 32];
        assert_eq!(
            resolve_listener_process(&candidate).unwrap_err(),
            ConductorProcessError::ExecutableDigestMismatch
        );
    }

    #[test]
    fn durable_fence_has_no_future_authority_lease() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let candidate = binding(listener.local_addr().unwrap());
        let pin = ConductorProcessBindingPin::for_binding(&candidate).unwrap();
        let dir = temp_dir();
        let store = TrustedConductorProcessStore::new(dir.join("process.json")).unwrap();
        store
            .bootstrap_from_out_of_band_pin(candidate, &pin)
            .unwrap();
        let before = store.state_summary().unwrap();
        let guard = store.begin_fence().unwrap();
        assert_eq!(guard.admin_endpoint(), listener.local_addr().unwrap());
        let fence = guard.finish().unwrap();
        let after = store.state_summary().unwrap();
        assert!(after.state_generation > before.state_generation);
        assert!(fence.ended_at_ms() >= fence.started_at_ms());
        assert_eq!(fence.persisted_state_digest(), after.state_digest);
        assert_eq!(
            after.last_fence_snapshot_digest,
            Some(fence.process_snapshot_digest())
        );
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn second_bootstrap_cannot_rebind_process_policy() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let candidate = binding(listener.local_addr().unwrap());
        let pin = ConductorProcessBindingPin::for_binding(&candidate).unwrap();
        let dir = temp_dir();
        let store = TrustedConductorProcessStore::new(dir.join("process.json")).unwrap();
        store
            .bootstrap_from_out_of_band_pin(candidate.clone(), &pin)
            .unwrap();
        assert_eq!(
            store
                .bootstrap_from_out_of_band_pin(candidate, &pin)
                .unwrap_err(),
            ConductorProcessError::AlreadyInitialized
        );
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn state_self_digest_detects_unsynchronized_corruption() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let candidate = binding(listener.local_addr().unwrap());
        let pin = ConductorProcessBindingPin::for_binding(&candidate).unwrap();
        let dir = temp_dir();
        let store = TrustedConductorProcessStore::new(dir.join("process.json")).unwrap();
        store.bootstrap_from_out_of_band_pin(candidate, &pin).unwrap();
        let bytes = fs::read(&store.state_path).unwrap();
        let mut state: TrustedConductorProcessState = serde_json::from_slice(&bytes).unwrap();
        state.last_trusted_time_ms += 1;
        fs::write(&store.state_path, serde_json::to_vec(&state).unwrap()).unwrap();
        assert_eq!(
            store.state_summary().unwrap_err(),
            ConductorProcessError::StateDigestMismatch
        );
        let _ = fs::remove_dir_all(dir);
    }
}

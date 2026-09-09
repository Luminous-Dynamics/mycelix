// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Crash-durable, out-of-band pinned exact target CellId selection.
//!
//! The target CellId and loopback Holochain Admin endpoint are fixed at first-use
//! bootstrap and then owned by one secure local state path. Normal live qualification
//! accepts neither a target nor an endpoint from the caller. It directly queries the
//! pinned loopback Admin endpoint with `list_cell_ids`, requires the exact pinned
//! CellId to be live, samples time after that observation, persists the trusted clock
//! floor, and only then returns a non-deserializable target-selection capability.
//!
//! This proves local target selection under the owner-protected-filesystem + pinned
//! loopback-endpoint model. It does not prove conductor-process integrity, same-UID
//! arbitrary-write resistance, full-machine anti-rollback, coordinator code identity,
//! approved release identity, or effect authority.

#[cfg(not(unix))]
compile_error!("coordinator target-cell trusted state v0.1 requires a Unix host");

use holochain_client::AdminWebsocket;
use mycelix_authority_coordinator_deployment_composer::{
    CoordinatorDeploymentCompositionError, PROTOCOL_VERSION as COMPOSER_PROTOCOL_VERSION,
    TARGET_SELECTION_PROFILE, TargetCellSelection,
};
use mycelix_authority_coordinator_native_attestor::{LocalAdminEndpoint, NativeAttestorError};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::net::SocketAddr;
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-target-cell-v0.1";
pub const BINDING_PROFILE: &str =
    "mycelix-authority-coordinator-target-cell-binding-v1-blake3-framed";
pub const BINDING_PIN_PROFILE: &str =
    "mycelix-authority-coordinator-target-cell-binding-pin-v1-blake3";
pub const STATE_PROFILE: &str =
    "mycelix-authority-coordinator-target-cell-state-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-target-cell-qualification-v1-blake3-framed";
pub const STATE_FILE_MODE: u32 = 0o600;
pub const MAX_STATE_BYTES: usize = 1024 * 1024;
pub const MAX_LIVE_TARGET_REUSE_MS: u64 = 5_000;

const OBSERVER_ORIGIN: &str = "mycelix-authority-coordinator-target-cell-v0.1";
const DOMAIN_BINDING: &[u8] = b"mycelix/authority/coordinator-target-cell-binding/v1";
const DOMAIN_STATE: &[u8] = b"mycelix/authority/coordinator-target-cell-state/v1";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority/coordinator-target-cell-qualification/v1";
const DOMAIN_SELECTION_REF: &[u8] = b"mycelix/authority/coordinator-target-cell-selection-ref/v1";
const HOLO_HASH_RAW_LEN: usize = 39;
const MAX_TEXT_BYTES: usize = 2048;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TargetCellBinding {
    pub protocol_version: String,
    pub binding_id: String,
    pub dna_hash_raw_39: Vec<u8>,
    pub agent_pub_key_raw_39: Vec<u8>,
    pub admin_endpoint: SocketAddr,
    pub binding_ref: String,
}

impl TargetCellBinding {
    pub fn validate(&self) -> Result<(), TargetCellError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(TargetCellError::WrongProtocol);
        }
        validate_text(&self.binding_id, "target binding id")?;
        validate_holo_hash(&self.dna_hash_raw_39, "target DNA hash")?;
        validate_holo_hash(&self.agent_pub_key_raw_39, "target agent key")?;
        LocalAdminEndpoint::new(self.admin_endpoint)
            .map_err(|_| TargetCellError::InvalidAdminEndpoint)?;
        validate_text(&self.binding_ref, "target binding reference")?;
        Ok(())
    }

    pub fn binding_digest(&self) -> Result<[u8; 32], TargetCellError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_BINDING);
        frame(&mut h, PROTOCOL_VERSION.as_bytes());
        frame(&mut h, BINDING_PROFILE.as_bytes());
        frame(&mut h, self.binding_id.as_bytes());
        frame(&mut h, &self.dna_hash_raw_39);
        frame(&mut h, &self.agent_pub_key_raw_39);
        frame(&mut h, self.admin_endpoint.to_string().as_bytes());
        frame(&mut h, self.binding_ref.as_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TargetCellBindingPin {
    pub binding_digest: [u8; 32],
    pub binding_profile: String,
}

impl TargetCellBindingPin {
    pub fn validate(&self) -> Result<(), TargetCellError> {
        validate_digest(&self.binding_digest, "target binding pin digest")?;
        if self.binding_profile != BINDING_PROFILE {
            return Err(TargetCellError::WrongBindingProfile);
        }
        Ok(())
    }
}

/// Serialized state is data, not positive authority. Normal live qualification loads
/// this exact state from the store-owned path while holding the exclusive lock.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedTargetCellState {
    pub protocol_version: String,
    pub state_profile: String,
    pub state_generation: u64,
    pub previous_state_digest: Option<[u8; 32]>,
    pub binding: TargetCellBinding,
    pub binding_digest: [u8; 32],
    pub last_trusted_time_ms: u64,
    pub state_digest: [u8; 32],
}

impl TrustedTargetCellState {
    fn validate(&self) -> Result<(), TargetCellError> {
        if self.protocol_version != PROTOCOL_VERSION || self.state_profile != STATE_PROFILE {
            return Err(TargetCellError::WrongStateProtocol);
        }
        if self.state_generation == 0 {
            return Err(TargetCellError::InvalidStateGeneration);
        }
        match (self.state_generation, self.previous_state_digest) {
            (1, None) => {}
            (1, Some(_)) => return Err(TargetCellError::InvalidPreviousStateDigest),
            (_, Some(previous)) => validate_digest(&previous, "previous target-state digest")?,
            (_, None) => return Err(TargetCellError::InvalidPreviousStateDigest),
        }
        self.binding.validate()?;
        if self.binding.binding_digest()? != self.binding_digest {
            return Err(TargetCellError::BindingDigestMismatch);
        }
        if self.last_trusted_time_ms == 0 {
            return Err(TargetCellError::InvalidTrustedClockFloor);
        }
        if self.compute_digest()? != self.state_digest {
            return Err(TargetCellError::StateDigestMismatch);
        }
        Ok(())
    }

    fn compute_digest(&self) -> Result<[u8; 32], TargetCellError> {
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
        frame(&mut h, BINDING_PROFILE.as_bytes());
        frame(&mut h, &self.last_trusted_time_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }

    fn advance_time(&self, trusted_time_ms: u64) -> Result<Self, TargetCellError> {
        if trusted_time_ms < self.last_trusted_time_ms {
            return Err(TargetCellError::ClockRollback {
                stored_ms: self.last_trusted_time_ms,
                observed_ms: trusted_time_ms,
            });
        }
        let state_generation = self
            .state_generation
            .checked_add(1)
            .ok_or(TargetCellError::StateGenerationOverflow)?;
        let mut next = Self {
            protocol_version: PROTOCOL_VERSION.into(),
            state_profile: STATE_PROFILE.into(),
            state_generation,
            previous_state_digest: Some(self.state_digest),
            binding: self.binding.clone(),
            binding_digest: self.binding_digest,
            last_trusted_time_ms: trusted_time_ms,
            state_digest: [0; 32],
        };
        next.state_digest = next.compute_digest()?;
        next.validate()?;
        Ok(next)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TargetCellStateSummary {
    pub state_generation: u64,
    pub state_digest: [u8; 32],
    pub binding_digest: [u8; 32],
    pub dna_hash_raw_39: Vec<u8>,
    pub agent_pub_key_raw_39: Vec<u8>,
    pub admin_endpoint: SocketAddr,
    pub last_trusted_time_ms: u64,
}

impl From<&TrustedTargetCellState> for TargetCellStateSummary {
    fn from(state: &TrustedTargetCellState) -> Self {
        Self {
            state_generation: state.state_generation,
            state_digest: state.state_digest,
            binding_digest: state.binding_digest,
            dna_hash_raw_39: state.binding.dna_hash_raw_39.clone(),
            agent_pub_key_raw_39: state.binding.agent_pub_key_raw_39.clone(),
            admin_endpoint: state.binding.admin_endpoint,
            last_trusted_time_ms: state.last_trusted_time_ms,
        }
    }
}

/// Non-deserializable positive target-selection capability.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedTargetCellSelection {
    selection: TargetCellSelection,
    binding_digest: [u8; 32],
    binding_profile: String,
    persisted_state_digest: [u8; 32],
    persisted_state_profile: String,
    observation_source_ref: String,
    qualification_digest: [u8; 32],
    qualification_profile: String,
    observed_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedTargetCellSelection {
    pub fn selection(&self) -> &TargetCellSelection {
        &self.selection
    }
    pub fn binding_digest(&self) -> [u8; 32] {
        self.binding_digest
    }
    pub fn binding_profile(&self) -> &str {
        &self.binding_profile
    }
    pub fn persisted_state_digest(&self) -> [u8; 32] {
        self.persisted_state_digest
    }
    pub fn persisted_state_profile(&self) -> &str {
        &self.persisted_state_profile
    }
    pub fn observation_source_ref(&self) -> &str {
        &self.observation_source_ref
    }
    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }
    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }
    pub fn observed_at_ms(&self) -> u64 {
        self.observed_at_ms
    }
    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }
}

#[derive(Clone, Debug)]
pub struct TrustedTargetCellStore {
    state_path: PathBuf,
    lock_path: PathBuf,
}

impl TrustedTargetCellStore {
    pub fn new(state_path: impl Into<PathBuf>) -> Result<Self, TargetCellError> {
        let requested = state_path.into();
        let file_name = requested
            .file_name()
            .ok_or(TargetCellError::StatePathHasNoFileName)?
            .to_os_string();
        let requested_parent = requested
            .parent()
            .ok_or(TargetCellError::StatePathHasNoParent)?;
        let anchored_parent = if requested_parent.is_absolute() {
            requested_parent.to_path_buf()
        } else {
            std::env::current_dir()
                .map_err(io_error("resolve current directory"))?
                .join(requested_parent)
        };
        let canonical_parent = fs::canonicalize(&anchored_parent)
            .map_err(io_error("canonicalize target-state parent"))?;
        require_secure_parent_directory(&canonical_parent)?;
        let state_path = canonical_parent.join(&file_name);
        let lock_name = format!("{}.lock", file_name.to_string_lossy());
        let lock_path = canonical_parent.join(lock_name);
        Ok(Self {
            state_path,
            lock_path,
        })
    }

    /// First-use ceremony only. The exact CellId + loopback endpoint must match an
    /// independently delivered binding fingerprint. No normal operation accepts a pin.
    pub fn bootstrap_from_out_of_band_pin(
        &self,
        binding: TargetCellBinding,
        pin: &TargetCellBindingPin,
    ) -> Result<TargetCellStateSummary, TargetCellError> {
        let _lock = self.acquire_exclusive_lock()?;
        self.require_state_absent()?;
        binding.validate()?;
        pin.validate()?;
        let binding_digest = binding.binding_digest()?;
        if pin.binding_digest != binding_digest || pin.binding_profile != BINDING_PROFILE {
            return Err(TargetCellError::BindingPinMismatch);
        }
        let now_ms = system_time_ms()?;
        let mut state = TrustedTargetCellState {
            protocol_version: PROTOCOL_VERSION.into(),
            state_profile: STATE_PROFILE.into(),
            state_generation: 1,
            previous_state_digest: None,
            binding,
            binding_digest,
            last_trusted_time_ms: now_ms,
            state_digest: [0; 32],
        };
        state.state_digest = state.compute_digest()?;
        state.validate()?;
        self.atomic_replace_state(&state)?;
        Ok((&state).into())
    }

    /// Query the state-owned pinned loopback Admin endpoint for all live CellIds and
    /// return positive target selection only if the exact pinned CellId is present.
    ///
    /// The public live API accepts neither a target CellId nor an endpoint.
    pub async fn qualify_live_target(&self) -> Result<QualifiedTargetCellSelection, TargetCellError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        let _started_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        let endpoint = LocalAdminEndpoint::new(state.binding.admin_endpoint)
            .map_err(|_| TargetCellError::InvalidAdminEndpoint)?;
        let admin = AdminWebsocket::connect(
            endpoint.socket_addr(),
            Some(OBSERVER_ORIGIN.to_string()),
        )
        .await
        .map_err(|error| TargetCellError::Conductor(error.to_string()))?;
        let live_cells = admin
            .list_cell_ids()
            .await
            .map_err(|error| TargetCellError::Conductor(error.to_string()))?;
        let observed_cells: Vec<(Vec<u8>, Vec<u8>)> = live_cells
            .iter()
            .map(|cell_id| {
                (
                    cell_id.dna_hash().get_raw_39().to_vec(),
                    cell_id.agent_pubkey().get_raw_39().to_vec(),
                )
            })
            .collect();
        let observed_at_ms = system_time_ms()?;
        self.finalize_live_observation(state, endpoint, &observed_cells, observed_at_ms)
    }

    pub fn state_summary(&self) -> Result<TargetCellStateSummary, TargetCellError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        Ok((&state).into())
    }

    fn finalize_live_observation(
        &self,
        state: TrustedTargetCellState,
        endpoint: LocalAdminEndpoint,
        observed_cells: &[(Vec<u8>, Vec<u8>)],
        observed_at_ms: u64,
    ) -> Result<QualifiedTargetCellSelection, TargetCellError> {
        if endpoint.socket_addr() != state.binding.admin_endpoint {
            return Err(TargetCellError::AdminEndpointMismatch);
        }
        if observed_at_ms < state.last_trusted_time_ms {
            return Err(TargetCellError::ClockRollback {
                stored_ms: state.last_trusted_time_ms,
                observed_ms: observed_at_ms,
            });
        }
        let mut matches = 0usize;
        for (dna, agent) in observed_cells {
            validate_holo_hash(dna, "observed live-cell DNA hash")?;
            validate_holo_hash(agent, "observed live-cell agent key")?;
            if dna == &state.binding.dna_hash_raw_39 && agent == &state.binding.agent_pub_key_raw_39 {
                matches = matches
                    .checked_add(1)
                    .ok_or(TargetCellError::DuplicateTargetCell)?;
            }
        }
        match matches {
            0 => return Err(TargetCellError::PinnedTargetNotLive),
            1 => {}
            _ => return Err(TargetCellError::DuplicateTargetCell),
        }

        let valid_until_ms = observed_at_ms
            .checked_add(MAX_LIVE_TARGET_REUSE_MS)
            .ok_or(TargetCellError::ClockOverflow)?;

        // The trusted clock floor is advanced durably before positive target authority
        // escapes this adapter.
        let next = state.advance_time(observed_at_ms)?;
        self.atomic_replace_state(&next)?;

        let selection_ref = selection_ref(&next, endpoint, observed_at_ms);
        let selection = TargetCellSelection {
            protocol_version: COMPOSER_PROTOCOL_VERSION.into(),
            dna_hash_raw_39: next.binding.dna_hash_raw_39.clone(),
            agent_pub_key_raw_39: next.binding.agent_pub_key_raw_39.clone(),
            selection_ref,
        };
        let selection_digest = selection
            .selection_digest()
            .map_err(TargetCellError::Composer)?;
        let observation_source_ref = format!(
            "admin-websocket:loopback:list-cell-ids:{}",
            endpoint.socket_addr()
        );
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_QUALIFICATION);
        frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
        frame(&mut h, &next.binding_digest);
        frame(&mut h, BINDING_PROFILE.as_bytes());
        frame(&mut h, &next.state_digest);
        frame(&mut h, STATE_PROFILE.as_bytes());
        frame(&mut h, &selection_digest);
        frame(&mut h, TARGET_SELECTION_PROFILE.as_bytes());
        frame(&mut h, observation_source_ref.as_bytes());
        frame(&mut h, &observed_at_ms.to_le_bytes());
        frame(&mut h, &valid_until_ms.to_le_bytes());
        let qualification_digest = *h.finalize().as_bytes();

        Ok(QualifiedTargetCellSelection {
            selection,
            binding_digest: next.binding_digest,
            binding_profile: BINDING_PROFILE.into(),
            persisted_state_digest: next.state_digest,
            persisted_state_profile: STATE_PROFILE.into(),
            observation_source_ref,
            qualification_digest,
            qualification_profile: QUALIFICATION_PROFILE.into(),
            observed_at_ms,
            valid_until_ms,
        })
    }

    fn acquire_exclusive_lock(&self) -> Result<File, TargetCellError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(TargetCellError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.lock_path)?;
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(STATE_FILE_MODE)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.lock_path)
            .map_err(io_error("open target-state lock"))?;
        require_secure_regular_file(&lock, "target-state lock")?;
        lock.lock()
            .map_err(io_error("acquire target-state exclusive lock"))?;
        Ok(lock)
    }

    fn require_state_absent(&self) -> Result<(), TargetCellError> {
        match fs::symlink_metadata(&self.state_path) {
            Ok(_) => Err(TargetCellError::AlreadyInitialized),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(TargetCellError::Io {
                operation: "inspect target bootstrap state path",
                message: error.to_string(),
            }),
        }
    }

    fn load_state_unlocked(&self) -> Result<TrustedTargetCellState, TargetCellError> {
        reject_symlink_if_present(&self.state_path)?;
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.state_path)
            .map_err(io_error("open target trusted state"))?;
        require_secure_regular_file(&file, "target trusted state")?;
        let len = file
            .metadata()
            .map_err(io_error("stat target trusted state"))?
            .len();
        if len == 0 || len > MAX_STATE_BYTES as u64 {
            return Err(TargetCellError::InvalidStateSize(len));
        }
        let mut bytes = Vec::with_capacity(len as usize);
        file.take((MAX_STATE_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(io_error("read target trusted state"))?;
        if bytes.len() > MAX_STATE_BYTES {
            return Err(TargetCellError::InvalidStateSize(bytes.len() as u64));
        }
        let state: TrustedTargetCellState = serde_json::from_slice(&bytes)
            .map_err(|error| TargetCellError::Json(error.to_string()))?;
        state.validate()?;
        Ok(state)
    }

    fn atomic_replace_state(&self, state: &TrustedTargetCellState) -> Result<(), TargetCellError> {
        state.validate()?;
        let parent = self
            .state_path
            .parent()
            .ok_or(TargetCellError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.state_path)?;
        let bytes = serde_json::to_vec(state)
            .map_err(|error| TargetCellError::Json(error.to_string()))?;
        if bytes.len() + 1 > MAX_STATE_BYTES {
            return Err(TargetCellError::InvalidStateSize((bytes.len() + 1) as u64));
        }
        let temp_path = self.unique_temp_path(state.state_generation)?;
        let result = (|| {
            let mut file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .mode(STATE_FILE_MODE)
                .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
                .open(&temp_path)
                .map_err(io_error("create target-state temp"))?;
            require_secure_regular_file(&file, "target-state temp")?;
            file.write_all(&bytes)
                .map_err(io_error("write target-state temp"))?;
            file.write_all(b"\n")
                .map_err(io_error("finish target-state temp"))?;
            file.sync_all()
                .map_err(io_error("fsync target-state temp"))?;
            drop(file);
            fs::rename(&temp_path, &self.state_path)
                .map_err(io_error("atomically replace target trusted state"))?;
            let directory = OpenOptions::new()
                .read(true)
                .custom_flags(libc::O_DIRECTORY | libc::O_CLOEXEC)
                .open(parent)
                .map_err(io_error("open target-state directory"))?;
            directory
                .sync_all()
                .map_err(io_error("fsync target-state directory"))?;
            Ok(())
        })();
        if result.is_err() {
            let _ = fs::remove_file(&temp_path);
        }
        result
    }

    fn unique_temp_path(&self, generation: u64) -> Result<PathBuf, TargetCellError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(TargetCellError::StatePathHasNoParent)?;
        let base = self
            .state_path
            .file_name()
            .ok_or(TargetCellError::StatePathHasNoFileName)?
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
        Err(TargetCellError::TemporaryPathExhausted)
    }
}

fn selection_ref(
    state: &TrustedTargetCellState,
    endpoint: LocalAdminEndpoint,
    observed_at_ms: u64,
) -> String {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_SELECTION_REF);
    frame(&mut h, &state.binding_digest);
    frame(&mut h, &state.state_digest);
    frame(&mut h, endpoint.socket_addr().to_string().as_bytes());
    frame(&mut h, &observed_at_ms.to_le_bytes());
    format!("trusted-target-cell-blake3:{}", encode_hex(h.finalize().as_bytes()))
}

fn checked_time_after_floor(floor_ms: u64) -> Result<u64, TargetCellError> {
    let now_ms = system_time_ms()?;
    if now_ms < floor_ms {
        Err(TargetCellError::ClockRollback {
            stored_ms: floor_ms,
            observed_ms: now_ms,
        })
    } else {
        Ok(now_ms)
    }
}

fn system_time_ms() -> Result<u64, TargetCellError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| TargetCellError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| TargetCellError::ClockOverflow)
}

fn reject_symlink_if_present(path: &Path) -> Result<(), TargetCellError> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            Err(TargetCellError::SymlinkNotAllowed(path.to_path_buf()))
        }
        Ok(_) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(TargetCellError::Io {
            operation: "inspect target-state path",
            message: error.to_string(),
        }),
    }
}

fn require_secure_parent_directory(path: &Path) -> Result<(), TargetCellError> {
    let metadata = fs::symlink_metadata(path).map_err(io_error("stat target-state directory"))?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(TargetCellError::InsecureParentDirectory);
    }
    if metadata.uid() != effective_uid() {
        return Err(TargetCellError::WrongOwner("target-state directory"));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode & 0o077 != 0 || mode & 0o200 == 0 {
        return Err(TargetCellError::InsecureDirectoryPermissions(mode));
    }
    Ok(())
}

fn require_secure_regular_file(file: &File, label: &'static str) -> Result<(), TargetCellError> {
    let metadata = file.metadata().map_err(io_error("stat target-state file"))?;
    if !metadata.is_file() {
        return Err(TargetCellError::NotRegularFile(label));
    }
    if metadata.uid() != effective_uid() {
        return Err(TargetCellError::WrongOwner(label));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != STATE_FILE_MODE {
        return Err(TargetCellError::InsecureFilePermissions { label, mode });
    }
    Ok(())
}

fn effective_uid() -> u32 {
    // SAFETY: geteuid takes no arguments and does not access caller memory.
    unsafe { libc::geteuid() }
}

fn validate_holo_hash(value: &[u8], field: &'static str) -> Result<(), TargetCellError> {
    if value.len() != HOLO_HASH_RAW_LEN || value.iter().all(|byte| *byte == 0) {
        Err(TargetCellError::InvalidHoloHash(field))
    } else {
        Ok(())
    }
}

fn validate_digest(value: &[u8; 32], field: &'static str) -> Result<(), TargetCellError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(TargetCellError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), TargetCellError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(TargetCellError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

fn encode_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn io_error(operation: &'static str) -> impl FnOnce(std::io::Error) -> TargetCellError {
    move |error| TargetCellError::Io {
        operation,
        message: error.to_string(),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TargetCellError {
    WrongProtocol,
    WrongBindingProfile,
    WrongStateProtocol,
    InvalidText(&'static str),
    InvalidHoloHash(&'static str),
    InvalidDigest(&'static str),
    InvalidAdminEndpoint,
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
    Conductor(String),
    AdminEndpointMismatch,
    PinnedTargetNotLive,
    DuplicateTargetCell,
    Composer(CoordinatorDeploymentCompositionError),
    Io { operation: &'static str, message: String },
    Json(String),
}

impl fmt::Display for TargetCellError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for TargetCellError {}

impl From<NativeAttestorError> for TargetCellError {
    fn from(error: NativeAttestorError) -> Self {
        Self::Conductor(error.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    static COUNTER: AtomicU64 = AtomicU64::new(1);

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn temp_dir() -> PathBuf {
        let id = COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "mycelix-target-cell-{}-{id}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir(&path).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o700)).unwrap();
        path
    }

    fn binding() -> TargetCellBinding {
        TargetCellBinding {
            protocol_version: PROTOCOL_VERSION.into(),
            binding_id: "authority-runtime-primary".into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            admin_endpoint: "127.0.0.1:30000".parse().unwrap(),
            binding_ref: "provisioning:authority-runtime-primary".into(),
        }
    }

    fn bootstrap(store: &TrustedTargetCellStore) -> TargetCellStateSummary {
        let binding = binding();
        let pin = TargetCellBindingPin {
            binding_digest: binding.binding_digest().unwrap(),
            binding_profile: BINDING_PROFILE.into(),
        };
        store.bootstrap_from_out_of_band_pin(binding, &pin).unwrap()
    }

    #[test]
    fn bootstrap_requires_exact_out_of_band_binding_pin() {
        let dir = temp_dir();
        let store = TrustedTargetCellStore::new(dir.join("target.json")).unwrap();
        let binding = binding();
        let wrong = TargetCellBindingPin {
            binding_digest: [77; 32],
            binding_profile: BINDING_PROFILE.into(),
        };
        assert_eq!(
            store
                .bootstrap_from_out_of_band_pin(binding, &wrong)
                .unwrap_err(),
            TargetCellError::BindingPinMismatch
        );
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn exact_live_cell_yields_non_retargetable_selection_after_persistence() {
        let dir = temp_dir();
        let store = TrustedTargetCellStore::new(dir.join("target.json")).unwrap();
        bootstrap(&store);
        let endpoint = LocalAdminEndpoint::new(binding().admin_endpoint).unwrap();
        let before = store.state_summary().unwrap();
        let observed_at = before.last_trusted_time_ms + 1;
        let lock = store.acquire_exclusive_lock().unwrap();
        let state = store.load_state_unlocked().unwrap();
        let qualified = store
            .finalize_live_observation(
                state,
                endpoint,
                &[(h(1), h(2)), (h(9), h(8))],
                observed_at,
            )
            .unwrap();
        drop(lock);
        let after = store.state_summary().unwrap();
        assert!(after.state_generation > before.state_generation);
        assert_eq!(qualified.selection().dna_hash_raw_39, h(9));
        assert_eq!(qualified.selection().agent_pub_key_raw_39, h(8));
        assert_eq!(qualified.observed_at_ms(), observed_at);
        assert_eq!(
            qualified.valid_until_ms(),
            observed_at + MAX_LIVE_TARGET_REUSE_MS
        );
        assert_eq!(qualified.persisted_state_digest(), after.state_digest);
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn missing_or_duplicate_pinned_cell_denies() {
        let dir = temp_dir();
        let store = TrustedTargetCellStore::new(dir.join("target.json")).unwrap();
        bootstrap(&store);
        let endpoint = LocalAdminEndpoint::new(binding().admin_endpoint).unwrap();

        let lock = store.acquire_exclusive_lock().unwrap();
        let state = store.load_state_unlocked().unwrap();
        let observed_at = state.last_trusted_time_ms + 1;
        assert_eq!(
            store
                .finalize_live_observation(state, endpoint, &[(h(1), h(2))], observed_at)
                .unwrap_err(),
            TargetCellError::PinnedTargetNotLive
        );
        drop(lock);

        let lock = store.acquire_exclusive_lock().unwrap();
        let state = store.load_state_unlocked().unwrap();
        let observed_at = state.last_trusted_time_ms + 1;
        assert_eq!(
            store
                .finalize_live_observation(
                    state,
                    endpoint,
                    &[(h(9), h(8)), (h(9), h(8))],
                    observed_at,
                )
                .unwrap_err(),
            TargetCellError::DuplicateTargetCell
        );
        drop(lock);
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn trusted_clock_floor_denies_observation_rollback() {
        let dir = temp_dir();
        let store = TrustedTargetCellStore::new(dir.join("target.json")).unwrap();
        bootstrap(&store);
        let endpoint = LocalAdminEndpoint::new(binding().admin_endpoint).unwrap();
        let lock = store.acquire_exclusive_lock().unwrap();
        let state = store.load_state_unlocked().unwrap();
        assert_eq!(
            store
                .finalize_live_observation(
                    state.clone(),
                    endpoint,
                    &[(h(9), h(8))],
                    state.last_trusted_time_ms - 1,
                )
                .unwrap_err(),
            TargetCellError::ClockRollback {
                stored_ms: state.last_trusted_time_ms,
                observed_ms: state.last_trusted_time_ms - 1,
            }
        );
        drop(lock);
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn state_self_digest_detects_unsynchronized_file_corruption() {
        let dir = temp_dir();
        let path = dir.join("target.json");
        let store = TrustedTargetCellStore::new(&path).unwrap();
        bootstrap(&store);
        let canonical_path = store.state_path.clone();
        let bytes = fs::read(&canonical_path).unwrap();
        let mut state: TrustedTargetCellState = serde_json::from_slice(&bytes).unwrap();
        state.last_trusted_time_ms += 1;
        fs::write(&canonical_path, serde_json::to_vec(&state).unwrap()).unwrap();
        assert_eq!(store.state_summary().unwrap_err(), TargetCellError::StateDigestMismatch);
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn relative_parent_is_resolved_once_to_canonical_real_directory() {
        let dir = temp_dir();
        let canonical = fs::canonicalize(&dir).unwrap();
        let store = TrustedTargetCellStore::new(dir.join("target.json")).unwrap();
        assert_eq!(store.state_path.parent().unwrap(), canonical.as_path());
        assert_eq!(store.lock_path.parent().unwrap(), canonical.as_path());
        let _ = fs::remove_dir_all(dir);
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Crash-durable latest complete coordinator release-registry state.
//!
//! Candidate registry snapshots are verified and durably advanced independently of
//! any release query. `qualify_current_release` accepts only an already-authenticated
//! release and obtains the current complete snapshot from one fixed state-owned path.
//! No caller-supplied previous snapshot, root, root pin, head proof or status proof
//! participates in normal current-release qualification.
//!
//! This establishes local crash/restart continuity under normal Unix host/filesystem
//! protection. Persisted self-digests detect accidental corruption, but they are not
//! a keyed anti-tamper or full-machine anti-rollback anchor. A same-UID arbitrary file
//! rewrite or restored machine image requires a separate TPM/hardware/enterprise/
//! witness theorem.

#[cfg(not(unix))]
compile_error!("coordinator release-registry trusted state v0.1 requires a Unix host");

use ed25519_dalek::{Signature as EdSignature, VerifyingKey as EdVerifyingKey};
use ml_dsa::signature::Verifier as MlVerifier;
use ml_dsa::{
    EncodedSignature, EncodedVerifyingKey, KeyInit as _, MlDsa65,
    Signature as MlSignature, VerifyingKey as MlVerifyingKey,
};
use mycelix_authority_coordinator_release::QualifiedCoordinatorReleaseRequirement;
use mycelix_authority_coordinator_release_currentness::{
    CoordinatorReleaseStatus, QualifiedCurrentCoordinatorRelease, REGISTRY_HEAD_PROFILE,
    STATUS_RECORD_PROFILE, VerifiedCoordinatorReleaseStatusAtHeadProof,
    VerifiedCurrentReleaseRegistryHeadProof, qualify_current_coordinator_release,
};
use mycelix_authority_coordinator_release_registry::{
    CoordinatorReleaseRegistryRootPin, CoordinatorReleaseRegistrySnapshot,
    CoordinatorReleaseRegistryTrustRoot, HybridRegistryThresholdSignature,
    HybridRegistryTrustKey, ROOT_PROFILE, THRESHOLD_SIGNATURE_PROFILE,
    bootstrap_registry_root, root_rotation_signature_message, snapshot_signature_message,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::ffi::OsString;
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-release-registry-state-v0.1";
pub const STATE_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-state-v1-blake3-framed";
pub const SNAPSHOT_CHECKPOINT_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-checkpoint-v1-blake3-framed";
pub const VERIFIER_REF_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-state-verifier-ref-v1-blake3";
pub const STATE_FILE_MODE: u32 = 0o600;
pub const MAX_STATE_BYTES: usize = 32 * 1024 * 1024;

const DOMAIN_STATE: &[u8] = b"mycelix/authority/coordinator-release-registry-state/v1";
const DOMAIN_CHECKPOINT: &[u8] =
    b"mycelix/authority/coordinator-release-registry-checkpoint/v1";
const DOMAIN_SIGNER_SET: &[u8] =
    b"mycelix/authority/coordinator-release-registry-state-signer-set/v1";
const DOMAIN_VERIFIER_REF: &[u8] =
    b"mycelix/authority/coordinator-release-registry-state-verifier-ref/v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedRegistrySnapshotCheckpoint {
    pub snapshot: CoordinatorReleaseRegistrySnapshot,
    pub registry_head_digest: [u8; 32],
    pub signer_set_digest: [u8; 32],
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl TrustedRegistrySnapshotCheckpoint {
    fn validate(&self) -> Result<(), TrustedRegistryStateError> {
        self.snapshot
            .validate()
            .map_err(|error| TrustedRegistryStateError::Registry(error.to_string()))?;
        let digest = self
            .snapshot
            .head_digest()
            .map_err(|error| TrustedRegistryStateError::Registry(error.to_string()))?;
        if digest != self.registry_head_digest {
            return Err(TrustedRegistryStateError::SnapshotDigestMismatch);
        }
        validate_digest(&self.signer_set_digest, "registry signer-set digest")?;
        if self.verified_at_ms == 0
            || self.verified_at_ms < self.snapshot.valid_from_ms
            || self.verified_at_ms >= self.snapshot.valid_until_ms
            || self.valid_until_ms <= self.verified_at_ms
            || self.valid_until_ms > self.snapshot.valid_until_ms
        {
            return Err(TrustedRegistryStateError::InvalidSnapshotCheckpointWindow);
        }
        Ok(())
    }

    fn checkpoint_digest(&self) -> Result<[u8; 32], TrustedRegistryStateError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_CHECKPOINT);
        frame(&mut hasher, SNAPSHOT_CHECKPOINT_PROFILE.as_bytes());
        frame(&mut hasher, &self.registry_head_digest);
        frame(&mut hasher, REGISTRY_HEAD_PROFILE.as_bytes());
        frame(&mut hasher, &self.signer_set_digest);
        frame(&mut hasher, &self.verified_at_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Serialized state is data, not a positive capability. The live trust property comes
/// from loading this shape from the store-owned canonical path while holding the
/// exclusive store lock. The unkeyed digest is an integrity/corruption check under
/// the owner-protected-filesystem model; it is not a proof against arbitrary same-UID
/// rewrite or restoration of a whole older machine image.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedCoordinatorReleaseRegistryState {
    pub protocol_version: String,
    pub state_profile: String,
    pub state_generation: u64,
    pub previous_state_digest: Option<[u8; 32]>,
    pub bootstrap_root_digest: [u8; 32],
    pub bootstrap_root_profile: String,
    pub current_root: CoordinatorReleaseRegistryTrustRoot,
    pub current_root_digest: [u8; 32],
    pub root_threshold_floor: u16,
    pub registry_head_threshold_floor: u16,
    pub current_snapshot: Option<TrustedRegistrySnapshotCheckpoint>,
    pub last_trusted_time_ms: u64,
    pub state_digest: [u8; 32],
}

impl TrustedCoordinatorReleaseRegistryState {
    fn validate(&self) -> Result<(), TrustedRegistryStateError> {
        if self.protocol_version != PROTOCOL_VERSION || self.state_profile != STATE_PROFILE {
            return Err(TrustedRegistryStateError::WrongStateProtocol);
        }
        if self.state_generation == 0 {
            return Err(TrustedRegistryStateError::InvalidStateGeneration);
        }
        match (self.state_generation, self.previous_state_digest) {
            (1, None) => {}
            (1, Some(_)) => return Err(TrustedRegistryStateError::InvalidPreviousStateDigest),
            (_, Some(previous)) => validate_digest(&previous, "previous state digest")?,
            (_, None) => return Err(TrustedRegistryStateError::InvalidPreviousStateDigest),
        }

        validate_digest(&self.bootstrap_root_digest, "bootstrap registry root digest")?;
        if self.bootstrap_root_profile != ROOT_PROFILE {
            return Err(TrustedRegistryStateError::WrongRootProfile);
        }
        self.current_root
            .validate()
            .map_err(|error| TrustedRegistryStateError::Registry(error.to_string()))?;
        let current_root_digest = self
            .current_root
            .root_digest()
            .map_err(|error| TrustedRegistryStateError::Registry(error.to_string()))?;
        if current_root_digest != self.current_root_digest {
            return Err(TrustedRegistryStateError::CurrentRootDigestMismatch);
        }
        if self.current_root.root_version == 1
            && self.current_root_digest != self.bootstrap_root_digest
        {
            return Err(TrustedRegistryStateError::BootstrapRootMismatch);
        }
        if self.root_threshold_floor == 0
            || self.registry_head_threshold_floor == 0
            || self.current_root.root_threshold < self.root_threshold_floor
            || self.current_root.registry_head_threshold < self.registry_head_threshold_floor
        {
            return Err(TrustedRegistryStateError::ThresholdFloorViolation);
        }
        if self.last_trusted_time_ms == 0
            || self.last_trusted_time_ms >= self.current_root.valid_until_ms
        {
            return Err(TrustedRegistryStateError::InvalidTrustedClockFloor);
        }

        if let Some(checkpoint) = &self.current_snapshot {
            checkpoint.validate()?;
            if checkpoint.snapshot.registry_id != self.current_root.registry_id
                || checkpoint.snapshot.release_authority_ref
                    != self.current_root.release_authority_ref
                || checkpoint.snapshot.root_version > self.current_root.root_version
                || checkpoint.verified_at_ms > self.last_trusted_time_ms
            {
                return Err(TrustedRegistryStateError::SnapshotOutsideStateScope);
            }
            if checkpoint.snapshot.root_version == self.current_root.root_version
                && checkpoint.valid_until_ms > self.current_root.valid_until_ms
            {
                return Err(TrustedRegistryStateError::SnapshotOutsideStateScope);
            }
        }

        if self.compute_digest()? != self.state_digest {
            return Err(TrustedRegistryStateError::StateDigestMismatch);
        }
        Ok(())
    }

    fn compute_digest(&self) -> Result<[u8; 32], TrustedRegistryStateError> {
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_STATE);
        frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame(&mut hasher, STATE_PROFILE.as_bytes());
        frame(&mut hasher, &self.state_generation.to_le_bytes());
        match self.previous_state_digest {
            Some(previous) => {
                frame(&mut hasher, b"previous");
                frame(&mut hasher, &previous);
            }
            None => frame(&mut hasher, b"genesis"),
        }
        frame(&mut hasher, &self.bootstrap_root_digest);
        frame(&mut hasher, self.bootstrap_root_profile.as_bytes());
        frame(&mut hasher, &self.current_root_digest);
        frame(&mut hasher, ROOT_PROFILE.as_bytes());
        frame(&mut hasher, &self.root_threshold_floor.to_le_bytes());
        frame(
            &mut hasher,
            &self.registry_head_threshold_floor.to_le_bytes(),
        );
        match &self.current_snapshot {
            Some(checkpoint) => {
                frame(&mut hasher, b"snapshot");
                frame(&mut hasher, &checkpoint.checkpoint_digest()?);
            }
            None => frame(&mut hasher, b"no-snapshot"),
        }
        frame(&mut hasher, &self.last_trusted_time_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }

    fn advance(
        &self,
        current_root: CoordinatorReleaseRegistryTrustRoot,
        current_snapshot: Option<TrustedRegistrySnapshotCheckpoint>,
        trusted_time_ms: u64,
        root_threshold_floor: u16,
        registry_head_threshold_floor: u16,
    ) -> Result<Self, TrustedRegistryStateError> {
        if trusted_time_ms < self.last_trusted_time_ms {
            return Err(TrustedRegistryStateError::ClockRollback {
                stored_ms: self.last_trusted_time_ms,
                observed_ms: trusted_time_ms,
            });
        }
        if root_threshold_floor < self.root_threshold_floor
            || registry_head_threshold_floor < self.registry_head_threshold_floor
        {
            return Err(TrustedRegistryStateError::ThresholdFloorViolation);
        }
        let state_generation = self
            .state_generation
            .checked_add(1)
            .ok_or(TrustedRegistryStateError::StateGenerationOverflow)?;
        let current_root_digest = current_root
            .root_digest()
            .map_err(|error| TrustedRegistryStateError::Registry(error.to_string()))?;
        let mut next = Self {
            protocol_version: PROTOCOL_VERSION.into(),
            state_profile: STATE_PROFILE.into(),
            state_generation,
            previous_state_digest: Some(self.state_digest),
            bootstrap_root_digest: self.bootstrap_root_digest,
            bootstrap_root_profile: self.bootstrap_root_profile.clone(),
            current_root,
            current_root_digest,
            root_threshold_floor,
            registry_head_threshold_floor,
            current_snapshot,
            last_trusted_time_ms: trusted_time_ms,
            state_digest: [0; 32],
        };
        next.state_digest = next.compute_digest()?;
        next.validate()?;
        Ok(next)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrustedRegistryStateSummary {
    pub state_generation: u64,
    pub state_digest: [u8; 32],
    pub root_version: u64,
    pub root_digest: [u8; 32],
    pub root_threshold_floor: u16,
    pub registry_head_threshold_floor: u16,
    pub registry_generation: Option<u64>,
    pub registry_head_digest: Option<[u8; 32]>,
    pub record_count: Option<usize>,
    pub last_trusted_time_ms: u64,
}

impl From<&TrustedCoordinatorReleaseRegistryState> for TrustedRegistryStateSummary {
    fn from(state: &TrustedCoordinatorReleaseRegistryState) -> Self {
        Self {
            state_generation: state.state_generation,
            state_digest: state.state_digest,
            root_version: state.current_root.root_version,
            root_digest: state.current_root_digest,
            root_threshold_floor: state.root_threshold_floor,
            registry_head_threshold_floor: state.registry_head_threshold_floor,
            registry_generation: state
                .current_snapshot
                .as_ref()
                .map(|checkpoint| checkpoint.snapshot.registry_generation),
            registry_head_digest: state
                .current_snapshot
                .as_ref()
                .map(|checkpoint| checkpoint.registry_head_digest),
            record_count: state
                .current_snapshot
                .as_ref()
                .map(|checkpoint| checkpoint.snapshot.records.len()),
            last_trusted_time_ms: state.last_trusted_time_ms,
        }
    }
}

#[derive(Clone, Debug)]
pub struct TrustedCoordinatorReleaseRegistryStore {
    state_path: PathBuf,
    lock_path: PathBuf,
}

impl TrustedCoordinatorReleaseRegistryStore {
    /// Resolve the requested state path to one stable canonical parent directory at
    /// construction time. Relative paths are first anchored to the current directory.
    /// Subsequent operations use only this resolved real path, so changing an ancestor
    /// symlink cannot retarget an already-created store instance.
    pub fn new(state_path: impl Into<PathBuf>) -> Result<Self, TrustedRegistryStateError> {
        let requested = state_path.into();
        let absolute = if requested.is_absolute() {
            requested
        } else {
            std::env::current_dir()
                .map_err(io_error("resolve current directory for registry state"))?
                .join(requested)
        };
        let file_name = absolute
            .file_name()
            .ok_or(TrustedRegistryStateError::StatePathHasNoFileName)?
            .to_os_string();
        let requested_parent = absolute
            .parent()
            .ok_or(TrustedRegistryStateError::StatePathHasNoParent)?;
        let canonical_parent = fs::canonicalize(requested_parent)
            .map_err(io_error("canonicalize registry-state directory"))?;
        require_secure_parent_directory(&canonical_parent)?;

        let state_path = canonical_parent.join(&file_name);
        let mut lock_name: OsString = file_name;
        lock_name.push(".lock");
        let lock_path = canonical_parent.join(lock_name);
        Ok(Self {
            state_path,
            lock_path,
        })
    }

    /// First-use only. Normal operations never accept a root pin.
    pub fn bootstrap_from_out_of_band_pin(
        &self,
        root: CoordinatorReleaseRegistryTrustRoot,
        pin: &CoordinatorReleaseRegistryRootPin,
        root_signatures: &[HybridRegistryThresholdSignature],
    ) -> Result<TrustedRegistryStateSummary, TrustedRegistryStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        self.require_state_absent()?;
        let qualified = bootstrap_registry_root(root.clone(), pin, root_signatures)
            .map_err(|error| TrustedRegistryStateError::Registry(error.to_string()))?;
        let verified_at_ms = qualified.verified_at_ms();
        let mut state = TrustedCoordinatorReleaseRegistryState {
            protocol_version: PROTOCOL_VERSION.into(),
            state_profile: STATE_PROFILE.into(),
            state_generation: 1,
            previous_state_digest: None,
            bootstrap_root_digest: qualified.root_lineage_digest(),
            bootstrap_root_profile: ROOT_PROFILE.into(),
            current_root: root.clone(),
            current_root_digest: qualified.root_digest(),
            root_threshold_floor: root.root_threshold,
            registry_head_threshold_floor: root.registry_head_threshold,
            current_snapshot: None,
            last_trusted_time_ms: verified_at_ms,
            state_digest: [0; 32],
        };
        state.state_digest = state.compute_digest()?;
        state.validate()?;
        self.atomic_replace_state(&state)?;
        Ok((&state).into())
    }

    /// Rotate using the state-owned old root. Threshold floors ratchet upward.
    pub fn rotate_root(
        &self,
        new_root: CoordinatorReleaseRegistryTrustRoot,
        old_root_signatures: &[HybridRegistryThresholdSignature],
        new_root_signatures: &[HybridRegistryThresholdSignature],
    ) -> Result<TrustedRegistryStateSummary, TrustedRegistryStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        let started_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        let old_root = &state.current_root;
        old_root.validate().map_err(registry_error)?;
        new_root.validate().map_err(registry_error)?;
        if !root_live_at(old_root, started_at_ms) || !root_live_at(&new_root, started_at_ms) {
            return Err(TrustedRegistryStateError::RootNotLive);
        }
        if new_root.root_id != old_root.root_id
            || new_root.release_authority_ref != old_root.release_authority_ref
            || new_root.registry_id != old_root.registry_id
        {
            return Err(TrustedRegistryStateError::RootScopeChanged);
        }
        let expected_version = old_root
            .root_version
            .checked_add(1)
            .ok_or(TrustedRegistryStateError::RootVersionOverflow)?;
        if new_root.root_version != expected_version {
            return Err(TrustedRegistryStateError::InvalidRootRotationVersion);
        }
        if new_root.root_threshold < state.root_threshold_floor
            || new_root.registry_head_threshold < state.registry_head_threshold_floor
        {
            return Err(TrustedRegistryStateError::ThresholdFloorViolation);
        }

        let message =
            root_rotation_signature_message(old_root, &new_root).map_err(registry_error)?;
        verify_exact_threshold(
            &old_root.root_keys,
            old_root.root_threshold,
            old_root_signatures,
            &message,
            started_at_ms,
        )?;
        verify_exact_threshold(
            &new_root.root_keys,
            new_root.root_threshold,
            new_root_signatures,
            &message,
            started_at_ms,
        )?;
        let verified_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        ensure_signers_live(&old_root.root_keys, old_root_signatures, verified_at_ms)?;
        ensure_signers_live(&new_root.root_keys, new_root_signatures, verified_at_ms)?;
        if !root_live_at(old_root, verified_at_ms) || !root_live_at(&new_root, verified_at_ms) {
            return Err(TrustedRegistryStateError::RootExpiredDuringVerification);
        }

        let root_floor = state.root_threshold_floor.max(new_root.root_threshold);
        let head_floor = state
            .registry_head_threshold_floor
            .max(new_root.registry_head_threshold);
        let next = state.advance(
            new_root,
            state.current_snapshot.clone(),
            verified_at_ms,
            root_floor,
            head_floor,
        )?;
        self.atomic_replace_state(&next)?;
        Ok((&next).into())
    }

    /// Verify a candidate complete snapshot and durably make it the state-owned latest
    /// registry state. No release query is involved in this transition.
    pub fn advance_snapshot(
        &self,
        snapshot: CoordinatorReleaseRegistrySnapshot,
        signatures: &[HybridRegistryThresholdSignature],
    ) -> Result<TrustedRegistryStateSummary, TrustedRegistryStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        let started_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        let root = &state.current_root;
        if !root_live_at(root, started_at_ms) {
            return Err(TrustedRegistryStateError::RootNotLive);
        }
        snapshot.validate().map_err(registry_error)?;
        if snapshot.root_version != root.root_version
            || snapshot.registry_id != root.registry_id
            || snapshot.release_authority_ref != root.release_authority_ref
        {
            return Err(TrustedRegistryStateError::SnapshotOutsideRootScope);
        }
        if started_at_ms < snapshot.valid_from_ms || started_at_ms >= snapshot.valid_until_ms {
            return Err(TrustedRegistryStateError::SnapshotNotLive);
        }
        let head_digest = snapshot.head_digest().map_err(registry_error)?;
        self.verify_snapshot_lineage(&state, &snapshot, head_digest)?;

        let message = snapshot_signature_message(root, &snapshot).map_err(registry_error)?;
        verify_exact_threshold(
            &root.registry_head_keys,
            root.registry_head_threshold,
            signatures,
            &message,
            started_at_ms,
        )?;
        let verified_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        ensure_signers_live(&root.registry_head_keys, signatures, verified_at_ms)?;
        if !root_live_at(root, verified_at_ms) {
            return Err(TrustedRegistryStateError::RootExpiredDuringVerification);
        }
        if verified_at_ms < snapshot.valid_from_ms || verified_at_ms >= snapshot.valid_until_ms {
            return Err(TrustedRegistryStateError::SnapshotExpiredDuringVerification);
        }
        let valid_until_ms = root
            .valid_until_ms
            .min(snapshot.valid_until_ms)
            .min(signer_horizon(&root.registry_head_keys, signatures)?);
        if valid_until_ms <= verified_at_ms {
            return Err(TrustedRegistryStateError::EmptySnapshotWindow);
        }

        let checkpoint = TrustedRegistrySnapshotCheckpoint {
            snapshot,
            registry_head_digest: head_digest,
            signer_set_digest: signer_set_digest(signatures),
            verified_at_ms,
            valid_until_ms,
        };
        checkpoint.validate()?;
        let next = state.advance(
            root.clone(),
            Some(checkpoint),
            verified_at_ms,
            state.root_threshold_floor,
            state.registry_head_threshold_floor,
        )?;
        self.atomic_replace_state(&next)?;
        Ok((&next).into())
    }

    /// Qualify one authenticated release using only the state-owned latest complete
    /// snapshot. The API accepts no snapshot/head/status proof input.
    pub fn qualify_current_release(
        &self,
        release: QualifiedCoordinatorReleaseRequirement,
    ) -> Result<QualifiedCurrentCoordinatorRelease, TrustedRegistryStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        let now_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        let root = &state.current_root;
        if !root_live_at(root, now_ms) {
            return Err(TrustedRegistryStateError::RootNotLive);
        }
        let checkpoint = state
            .current_snapshot
            .as_ref()
            .ok_or(TrustedRegistryStateError::NoTrustedSnapshot)?;
        checkpoint.validate()?;
        if checkpoint.snapshot.root_version != root.root_version {
            return Err(TrustedRegistryStateError::SnapshotNotUnderCurrentRoot);
        }
        if checkpoint.verified_at_ms > now_ms || checkpoint.valid_until_ms <= now_ms {
            return Err(TrustedRegistryStateError::TrustedSnapshotNotLive);
        }
        if release.verified_at_ms() > now_ms || release.valid_until_ms() <= now_ms {
            return Err(TrustedRegistryStateError::AuthenticatedReleaseNotLive);
        }

        let manifest = release.manifest();
        if manifest.release_authority_ref != checkpoint.snapshot.release_authority_ref
            || manifest.release_policy_digest != checkpoint.snapshot.release_policy_digest
            || manifest.release_policy_profile != checkpoint.snapshot.release_policy_profile
        {
            return Err(TrustedRegistryStateError::ReleaseOutsideRegistryScope);
        }
        let record = checkpoint
            .snapshot
            .records
            .binary_search_by_key(&release.manifest_digest(), |record| record.manifest_digest)
            .ok()
            .and_then(|index| checkpoint.snapshot.records.get(index))
            .ok_or(TrustedRegistryStateError::ReleaseMissingFromCompleteSnapshot)?;
        if record.status != CoordinatorReleaseStatus::Active {
            return Err(TrustedRegistryStateError::ReleaseNotActive(record.status));
        }

        // Ratchet the trusted host-clock floor and durably persist it BEFORE current-
        // release authority escapes. This prevents a later local clock rollback from
        // reusing this authorization time without detection under the filesystem model.
        let next = state.advance(
            root.clone(),
            Some(checkpoint.clone()),
            now_ms,
            state.root_threshold_floor,
            state.registry_head_threshold_floor,
        )?;
        self.atomic_replace_state(&next)?;

        let verifier_ref = verifier_ref(&next);
        let head_proof = VerifiedCurrentReleaseRegistryHeadProof {
            protocol_version:
                mycelix_authority_coordinator_release_currentness::PROTOCOL_VERSION.into(),
            release_policy_digest: checkpoint.snapshot.release_policy_digest,
            release_policy_profile: checkpoint.snapshot.release_policy_profile.clone(),
            registry_generation: checkpoint.snapshot.registry_generation,
            registry_head_digest: checkpoint.registry_head_digest,
            registry_head_profile: REGISTRY_HEAD_PROFILE.into(),
            source_ref: format!(
                "trusted-complete-registry-head-blake3:{}",
                encode_hex(&checkpoint.registry_head_digest)
            ),
            verifier_ref: format!("{verifier_ref}:head"),
            verified_at_ms: checkpoint.verified_at_ms,
            valid_until_ms: checkpoint.valid_until_ms,
        };
        let status_proof = VerifiedCoordinatorReleaseStatusAtHeadProof {
            protocol_version:
                mycelix_authority_coordinator_release_currentness::PROTOCOL_VERSION.into(),
            manifest_digest: record.manifest_digest,
            manifest_profile: record.manifest_profile.clone(),
            release_policy_digest: checkpoint.snapshot.release_policy_digest,
            release_policy_profile: checkpoint.snapshot.release_policy_profile.clone(),
            registry_generation: checkpoint.snapshot.registry_generation,
            registry_head_digest: checkpoint.registry_head_digest,
            registry_head_profile: REGISTRY_HEAD_PROFILE.into(),
            status: record.status,
            status_record_digest: record.status_record_digest().map_err(registry_error)?,
            status_record_profile: STATUS_RECORD_PROFILE.into(),
            verifier_ref: format!("{verifier_ref}:local-status"),
            verified_at_ms: checkpoint.verified_at_ms,
            valid_until_ms: checkpoint.valid_until_ms,
        };
        qualify_current_coordinator_release(release, &head_proof, &status_proof, now_ms)
            .map_err(|error| TrustedRegistryStateError::Currentness(error.to_string()))
    }

    /// Diagnostics only; this summary is not positive authority.
    pub fn state_summary(&self) -> Result<TrustedRegistryStateSummary, TrustedRegistryStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        Ok((&state).into())
    }

    fn verify_snapshot_lineage(
        &self,
        state: &TrustedCoordinatorReleaseRegistryState,
        snapshot: &CoordinatorReleaseRegistrySnapshot,
        head_digest: [u8; 32],
    ) -> Result<(), TrustedRegistryStateError> {
        match &state.current_snapshot {
            None => {
                if state.current_root.root_version != 1
                    || snapshot.registry_generation != 1
                    || snapshot.previous_registry_head_digest.is_some()
                {
                    return Err(TrustedRegistryStateError::InvalidSnapshotGenesis);
                }
            }
            Some(current) if current.registry_head_digest == head_digest => {
                if current.snapshot.registry_generation != snapshot.registry_generation {
                    return Err(TrustedRegistryStateError::SnapshotRevalidationMismatch);
                }
            }
            Some(current) => {
                let expected = current
                    .snapshot
                    .registry_generation
                    .checked_add(1)
                    .ok_or(TrustedRegistryStateError::RegistryGenerationOverflow)?;
                if snapshot.registry_generation != expected
                    || snapshot.previous_registry_head_digest != Some(current.registry_head_digest)
                    || snapshot.root_version < current.snapshot.root_version
                    || snapshot.registry_id != current.snapshot.registry_id
                    || snapshot.release_authority_ref != current.snapshot.release_authority_ref
                    || snapshot.release_policy_digest != current.snapshot.release_policy_digest
                    || snapshot.release_policy_profile != current.snapshot.release_policy_profile
                {
                    return Err(TrustedRegistryStateError::SnapshotPredecessorMismatch);
                }
                require_monotone_records(&current.snapshot, snapshot)?;
            }
        }
        Ok(())
    }

    fn acquire_exclusive_lock(&self) -> Result<File, TrustedRegistryStateError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(TrustedRegistryStateError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.lock_path)?;
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(STATE_FILE_MODE)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.lock_path)
            .map_err(io_error("open registry-state lock"))?;
        require_secure_regular_file(&lock, "registry-state lock")?;
        lock.lock()
            .map_err(io_error("acquire registry-state exclusive lock"))?;
        Ok(lock)
    }

    fn require_state_absent(&self) -> Result<(), TrustedRegistryStateError> {
        match fs::symlink_metadata(&self.state_path) {
            Ok(_) => Err(TrustedRegistryStateError::AlreadyInitialized),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(TrustedRegistryStateError::Io {
                operation: "inspect registry bootstrap state path",
                message: error.to_string(),
            }),
        }
    }

    fn load_state_unlocked(
        &self,
    ) -> Result<TrustedCoordinatorReleaseRegistryState, TrustedRegistryStateError> {
        reject_symlink_if_present(&self.state_path)?;
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.state_path)
            .map_err(io_error("open registry trusted state"))?;
        require_secure_regular_file(&file, "registry trusted state")?;
        let len = file
            .metadata()
            .map_err(io_error("stat registry trusted state"))?
            .len();
        if len == 0 || len > MAX_STATE_BYTES as u64 {
            return Err(TrustedRegistryStateError::InvalidStateSize(len));
        }
        let mut bytes = Vec::with_capacity(len as usize);
        file.take((MAX_STATE_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(io_error("read registry trusted state"))?;
        if bytes.len() > MAX_STATE_BYTES {
            return Err(TrustedRegistryStateError::InvalidStateSize(
                bytes.len() as u64,
            ));
        }
        let state: TrustedCoordinatorReleaseRegistryState = serde_json::from_slice(&bytes)
            .map_err(|error| TrustedRegistryStateError::Json(error.to_string()))?;
        state.validate()?;
        Ok(state)
    }

    fn atomic_replace_state(
        &self,
        state: &TrustedCoordinatorReleaseRegistryState,
    ) -> Result<(), TrustedRegistryStateError> {
        state.validate()?;
        let parent = self
            .state_path
            .parent()
            .ok_or(TrustedRegistryStateError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.state_path)?;
        let bytes = serde_json::to_vec(state)
            .map_err(|error| TrustedRegistryStateError::Json(error.to_string()))?;
        if bytes.len() + 1 > MAX_STATE_BYTES {
            return Err(TrustedRegistryStateError::InvalidStateSize(
                (bytes.len() + 1) as u64,
            ));
        }

        let tmp_path = self.unique_temp_path(state.state_generation)?;
        let result = (|| {
            let mut file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .mode(STATE_FILE_MODE)
                .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
                .open(&tmp_path)
                .map_err(io_error("create registry-state temp"))?;
            require_secure_regular_file(&file, "registry-state temp")?;
            file.write_all(&bytes)
                .map_err(io_error("write registry-state temp"))?;
            file.write_all(b"\n")
                .map_err(io_error("finish registry-state temp"))?;
            file.sync_all()
                .map_err(io_error("fsync registry-state temp"))?;
            drop(file);
            fs::rename(&tmp_path, &self.state_path)
                .map_err(io_error("atomically replace registry trusted state"))?;
            let directory = OpenOptions::new()
                .read(true)
                .custom_flags(libc::O_DIRECTORY | libc::O_CLOEXEC)
                .open(parent)
                .map_err(io_error("open registry-state directory"))?;
            directory
                .sync_all()
                .map_err(io_error("fsync registry-state directory"))?;
            Ok(())
        })();
        if result.is_err() {
            let _ = fs::remove_file(&tmp_path);
        }
        result
    }

    fn unique_temp_path(&self, generation: u64) -> Result<PathBuf, TrustedRegistryStateError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(TrustedRegistryStateError::StatePathHasNoParent)?;
        let base = self
            .state_path
            .file_name()
            .ok_or(TrustedRegistryStateError::StatePathHasNoFileName)?
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
        Err(TrustedRegistryStateError::TemporaryPathExhausted)
    }
}

fn require_monotone_records(
    previous: &CoordinatorReleaseRegistrySnapshot,
    next: &CoordinatorReleaseRegistrySnapshot,
) -> Result<(), TrustedRegistryStateError> {
    let next_map: BTreeMap<[u8; 32], _> = next
        .records
        .iter()
        .map(|record| (record.manifest_digest, record))
        .collect();
    for old in &previous.records {
        let new = next_map
            .get(&old.manifest_digest)
            .copied()
            .ok_or(TrustedRegistryStateError::RegistryRecordRemoved)?;
        if new.manifest_profile != old.manifest_profile
            || new.status_effective_at_ms < old.status_effective_at_ms
        {
            return Err(TrustedRegistryStateError::RegistryRecordHistoryRewritten);
        }
        if new.status == old.status {
            if new.status_effective_at_ms != old.status_effective_at_ms
                || new.status_ref != old.status_ref
            {
                return Err(TrustedRegistryStateError::RegistryRecordHistoryRewritten);
            }
        } else if old.status != CoordinatorReleaseStatus::Active
            || new.status == CoordinatorReleaseStatus::Active
            || new.status_effective_at_ms <= old.status_effective_at_ms
        {
            return Err(TrustedRegistryStateError::InvalidStatusTransition);
        }
    }
    Ok(())
}

fn verify_exact_threshold(
    keys: &[HybridRegistryTrustKey],
    threshold: u16,
    signatures: &[HybridRegistryThresholdSignature],
    message: &[u8],
    now_ms: u64,
) -> Result<(), TrustedRegistryStateError> {
    if signatures.len() != usize::from(threshold) {
        return Err(TrustedRegistryStateError::ThresholdSignatureCountMismatch);
    }
    let mut signers = BTreeSet::new();
    for signature in signatures {
        signature.validate().map_err(registry_error)?;
        if signature.signature_profile != THRESHOLD_SIGNATURE_PROFILE {
            return Err(TrustedRegistryStateError::WrongThresholdSignatureProfile);
        }
        if !signers.insert(signature.key_id.clone()) {
            return Err(TrustedRegistryStateError::DuplicateThresholdSigner);
        }
        let key = keys
            .iter()
            .find(|key| key.key_id == signature.key_id)
            .ok_or(TrustedRegistryStateError::UnauthorizedThresholdSigner)?;
        if !trust_key_live_at(key, now_ms) {
            return Err(TrustedRegistryStateError::ThresholdSignerNotLive);
        }
        verify_hybrid(key, message, signature)?;
    }
    Ok(())
}

fn ensure_signers_live(
    keys: &[HybridRegistryTrustKey],
    signatures: &[HybridRegistryThresholdSignature],
    now_ms: u64,
) -> Result<(), TrustedRegistryStateError> {
    for signature in signatures {
        let key = keys
            .iter()
            .find(|key| key.key_id == signature.key_id)
            .ok_or(TrustedRegistryStateError::UnauthorizedThresholdSigner)?;
        if !trust_key_live_at(key, now_ms) {
            return Err(TrustedRegistryStateError::ThresholdSignerExpiredDuringVerification);
        }
    }
    Ok(())
}

fn signer_horizon(
    keys: &[HybridRegistryTrustKey],
    signatures: &[HybridRegistryThresholdSignature],
) -> Result<u64, TrustedRegistryStateError> {
    signatures
        .iter()
        .map(|signature| {
            keys.iter()
                .find(|key| key.key_id == signature.key_id)
                .map(|key| key.valid_until_ms)
                .ok_or(TrustedRegistryStateError::UnauthorizedThresholdSigner)
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .min()
        .ok_or(TrustedRegistryStateError::ThresholdSignatureCountMismatch)
}

fn signer_set_digest(signatures: &[HybridRegistryThresholdSignature]) -> [u8; 32] {
    let mut ids: Vec<_> = signatures
        .iter()
        .map(|signature| signature.key_id.as_str())
        .collect();
    ids.sort();
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_SIGNER_SET);
    for id in ids {
        frame(&mut hasher, id.as_bytes());
    }
    *hasher.finalize().as_bytes()
}

fn verify_hybrid(
    key: &HybridRegistryTrustKey,
    message: &[u8],
    signature: &HybridRegistryThresholdSignature,
) -> Result<(), TrustedRegistryStateError> {
    let ed_key: [u8; 32] = key
        .ed25519_public_key
        .as_slice()
        .try_into()
        .map_err(|_| TrustedRegistryStateError::InvalidEd25519PublicKey)?;
    let ed_signature: [u8; 64] = signature
        .ed25519_signature
        .as_slice()
        .try_into()
        .map_err(|_| TrustedRegistryStateError::InvalidEd25519Signature)?;
    let verifier = EdVerifyingKey::from_bytes(&ed_key)
        .map_err(|_| TrustedRegistryStateError::InvalidEd25519PublicKey)?;
    verifier
        .verify_strict(message, &EdSignature::from_bytes(&ed_signature))
        .map_err(|_| TrustedRegistryStateError::Ed25519VerificationFailed)?;

    let encoded_key = EncodedVerifyingKey::<MlDsa65>::try_from(key.ml_dsa_65_public_key.as_slice())
        .map_err(|_| TrustedRegistryStateError::InvalidMlDsa65PublicKey)?;
    let verifier = MlVerifyingKey::<MlDsa65>::decode(&encoded_key);
    let encoded_signature =
        EncodedSignature::<MlDsa65>::try_from(signature.ml_dsa_65_signature.as_slice())
            .map_err(|_| TrustedRegistryStateError::InvalidMlDsa65Signature)?;
    let signature = MlSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or(TrustedRegistryStateError::InvalidMlDsa65Signature)?;
    verifier
        .verify(message, &signature)
        .map_err(|_| TrustedRegistryStateError::MlDsa65VerificationFailed)
}

fn root_live_at(root: &CoordinatorReleaseRegistryTrustRoot, now_ms: u64) -> bool {
    root.valid_from_ms <= now_ms && now_ms < root.valid_until_ms
}

fn trust_key_live_at(key: &HybridRegistryTrustKey, now_ms: u64) -> bool {
    key.valid_from_ms <= now_ms && now_ms < key.valid_until_ms
}

fn checked_time_after_floor(floor_ms: u64) -> Result<u64, TrustedRegistryStateError> {
    let now_ms = system_time_ms()?;
    if now_ms < floor_ms {
        Err(TrustedRegistryStateError::ClockRollback {
            stored_ms: floor_ms,
            observed_ms: now_ms,
        })
    } else {
        Ok(now_ms)
    }
}

fn system_time_ms() -> Result<u64, TrustedRegistryStateError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| TrustedRegistryStateError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| TrustedRegistryStateError::ClockOverflow)
}

fn verifier_ref(state: &TrustedCoordinatorReleaseRegistryState) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_VERIFIER_REF);
    frame(&mut hasher, VERIFIER_REF_PROFILE.as_bytes());
    frame(&mut hasher, &state.state_digest);
    format!(
        "coordinator-release-registry-state-blake3:{}",
        encode_hex(hasher.finalize().as_bytes())
    )
}

fn reject_symlink_if_present(path: &Path) -> Result<(), TrustedRegistryStateError> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            Err(TrustedRegistryStateError::SymlinkNotAllowed(
                path.to_path_buf(),
            ))
        }
        Ok(_) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(TrustedRegistryStateError::Io {
            operation: "inspect registry-state path",
            message: error.to_string(),
        }),
    }
}

fn require_secure_parent_directory(path: &Path) -> Result<(), TrustedRegistryStateError> {
    let metadata =
        fs::symlink_metadata(path).map_err(io_error("stat registry-state directory"))?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(TrustedRegistryStateError::InsecureParentDirectory);
    }
    if metadata.uid() != effective_uid() {
        return Err(TrustedRegistryStateError::WrongOwner(
            "registry-state directory",
        ));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode & 0o077 != 0 || mode & 0o200 == 0 {
        return Err(TrustedRegistryStateError::InsecureDirectoryPermissions(
            mode,
        ));
    }
    Ok(())
}

fn require_secure_regular_file(
    file: &File,
    label: &'static str,
) -> Result<(), TrustedRegistryStateError> {
    let metadata = file
        .metadata()
        .map_err(io_error("stat registry-state file"))?;
    if !metadata.is_file() {
        return Err(TrustedRegistryStateError::NotRegularFile(label));
    }
    if metadata.uid() != effective_uid() {
        return Err(TrustedRegistryStateError::WrongOwner(label));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != STATE_FILE_MODE {
        return Err(TrustedRegistryStateError::InsecureFilePermissions {
            label,
            mode,
        });
    }
    Ok(())
}

fn effective_uid() -> u32 {
    // SAFETY: geteuid takes no arguments and does not access caller memory.
    unsafe { libc::geteuid() }
}

fn validate_digest(
    value: &[u8; 32],
    field: &'static str,
) -> Result<(), TrustedRegistryStateError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(TrustedRegistryStateError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
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

fn registry_error(error: impl ToString) -> TrustedRegistryStateError {
    TrustedRegistryStateError::Registry(error.to_string())
}

fn io_error(operation: &'static str) -> impl FnOnce(std::io::Error) -> TrustedRegistryStateError {
    move |error| TrustedRegistryStateError::Io {
        operation,
        message: error.to_string(),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TrustedRegistryStateError {
    WrongStateProtocol,
    WrongRootProfile,
    InvalidStateGeneration,
    StateGenerationOverflow,
    InvalidPreviousStateDigest,
    InvalidDigest(&'static str),
    CurrentRootDigestMismatch,
    BootstrapRootMismatch,
    ThresholdFloorViolation,
    InvalidTrustedClockFloor,
    SnapshotDigestMismatch,
    InvalidSnapshotCheckpointWindow,
    SnapshotOutsideStateScope,
    StateDigestMismatch,
    StatePathHasNoParent,
    StatePathHasNoFileName,
    AlreadyInitialized,
    InvalidStateSize(u64),
    TemporaryPathExhausted,
    SymlinkNotAllowed(PathBuf),
    InsecureParentDirectory,
    InsecureDirectoryPermissions(u32),
    InsecureFilePermissions {
        label: &'static str,
        mode: u32,
    },
    NotRegularFile(&'static str),
    WrongOwner(&'static str),
    ClockBeforeUnixEpoch,
    ClockOverflow,
    ClockRollback {
        stored_ms: u64,
        observed_ms: u64,
    },
    Registry(String),
    Currentness(String),
    RootNotLive,
    RootExpiredDuringVerification,
    RootScopeChanged,
    RootVersionOverflow,
    InvalidRootRotationVersion,
    SnapshotOutsideRootScope,
    SnapshotNotLive,
    SnapshotExpiredDuringVerification,
    EmptySnapshotWindow,
    InvalidSnapshotGenesis,
    SnapshotRevalidationMismatch,
    SnapshotPredecessorMismatch,
    RegistryGenerationOverflow,
    RegistryRecordRemoved,
    RegistryRecordHistoryRewritten,
    InvalidStatusTransition,
    NoTrustedSnapshot,
    SnapshotNotUnderCurrentRoot,
    TrustedSnapshotNotLive,
    AuthenticatedReleaseNotLive,
    ReleaseOutsideRegistryScope,
    ReleaseMissingFromCompleteSnapshot,
    ReleaseNotActive(CoordinatorReleaseStatus),
    WrongThresholdSignatureProfile,
    ThresholdSignatureCountMismatch,
    DuplicateThresholdSigner,
    UnauthorizedThresholdSigner,
    ThresholdSignerNotLive,
    ThresholdSignerExpiredDuringVerification,
    InvalidEd25519PublicKey,
    InvalidEd25519Signature,
    Ed25519VerificationFailed,
    InvalidMlDsa65PublicKey,
    InvalidMlDsa65Signature,
    MlDsa65VerificationFailed,
    Json(String),
    Io {
        operation: &'static str,
        message: String,
    },
}

impl fmt::Display for TrustedRegistryStateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for TrustedRegistryStateError {}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as EdSigner, SigningKey as EdSigningKey};
    use ml_dsa::signature::{Keypair as _, Signer as MlSigner};
    use ml_dsa::{Generate as _, KeyExport as _, SigningKey as MlSigningKey};
    use mycelix_authority_coordinator_deployment::CoordinatorCodeIdentity;
    use mycelix_authority_coordinator_release::{
        CoordinatorReleaseManifest, MANIFEST_PROFILE, PROTOCOL_VERSION as RELEASE_PROTOCOL,
        SIGNATURE_PROOF_PROTOCOL, VerifiedCoordinatorReleaseSignatureProof,
        qualify_coordinator_release,
    };
    use mycelix_authority_coordinator_release_registry::{
        CoordinatorReleaseRegistryRecord, PROTOCOL_VERSION as REGISTRY_PROTOCOL, ROOT_PROFILE,
        root_bootstrap_signature_message,
    };
    use std::sync::atomic::{AtomicU64, Ordering};

    static COUNTER: AtomicU64 = AtomicU64::new(1);

    struct Signer {
        id: String,
        ed: EdSigningKey,
        ml: MlSigningKey<MlDsa65>,
    }

    impl Signer {
        fn new(id: &str, seed: u8) -> Self {
            Self {
                id: id.into(),
                ed: EdSigningKey::from_bytes(&[seed; 32]),
                ml: MlSigningKey::<MlDsa65>::generate(),
            }
        }

        fn key(&self, now_ms: u64) -> HybridRegistryTrustKey {
            HybridRegistryTrustKey {
                key_id: self.id.clone(),
                ed25519_public_key: self.ed.verifying_key().to_bytes().to_vec(),
                ml_dsa_65_public_key: self.ml.verifying_key().encode().as_slice().to_vec(),
                valid_from_ms: now_ms.saturating_sub(5_000),
                valid_until_ms: now_ms + 180_000,
            }
        }

        fn sign(&self, message: &[u8]) -> HybridRegistryThresholdSignature {
            HybridRegistryThresholdSignature {
                signature_profile: THRESHOLD_SIGNATURE_PROFILE.into(),
                key_id: self.id.clone(),
                ed25519_signature: EdSigner::sign(&self.ed, message).to_bytes().to_vec(),
                ml_dsa_65_signature: MlSigner::sign(&self.ml, message)
                    .encode()
                    .as_slice()
                    .to_vec(),
            }
        }
    }

    fn temp_dir() -> PathBuf {
        let id = COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "mycelix-registry-state-{}-{id}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir(&path).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o700)).unwrap();
        path
    }

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn holo(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn root(
        now_ms: u64,
        roots: &[&Signer],
        heads: &[&Signer],
        version: u64,
        root_threshold: u16,
        head_threshold: u16,
    ) -> CoordinatorReleaseRegistryTrustRoot {
        CoordinatorReleaseRegistryTrustRoot {
            protocol_version: REGISTRY_PROTOCOL.into(),
            root_id: "registry-root".into(),
            root_version: version,
            release_authority_ref: "release-authority:offline".into(),
            registry_id: "coordinator-release-registry".into(),
            root_keys: roots.iter().map(|signer| signer.key(now_ms)).collect(),
            root_threshold,
            registry_head_keys: heads.iter().map(|signer| signer.key(now_ms)).collect(),
            registry_head_threshold: head_threshold,
            valid_from_ms: now_ms.saturating_sub(5_000),
            valid_until_ms: now_ms + 180_000,
        }
    }

    fn bootstrap(
        store: &TrustedCoordinatorReleaseRegistryStore,
        candidate: CoordinatorReleaseRegistryTrustRoot,
        signers: &[&Signer],
    ) {
        let pin = CoordinatorReleaseRegistryRootPin {
            root_digest: candidate.root_digest().unwrap(),
            root_profile: ROOT_PROFILE.into(),
        };
        let message = root_bootstrap_signature_message(&candidate).unwrap();
        let signatures: Vec<_> = signers
            .iter()
            .take(usize::from(candidate.root_threshold))
            .map(|signer| signer.sign(&message))
            .collect();
        store
            .bootstrap_from_out_of_band_pin(candidate, &pin, &signatures)
            .unwrap();
    }

    fn manifest(now_ms: u64, id: u8) -> CoordinatorReleaseManifest {
        CoordinatorReleaseManifest {
            protocol_version: RELEASE_PROTOCOL.into(),
            release_id: format!("release-{id}"),
            release_version: u64::from(id).max(1),
            dna_hash_raw_39: holo(9),
            coordinators: vec![CoordinatorCodeIdentity {
                zome_name: "authority_current_freshness_verifier".into(),
                wasm_hash_raw_39: holo(1),
            }],
            dna_bundle_digest: d(2),
            source_tree_digest: d(3),
            lockfile_digest: d(4),
            toolchain_digest: d(5),
            build_recipe_digest: d(6),
            sbom_digest: d(7),
            source_ref: "git:tree".into(),
            build_ref: "nix:build".into(),
            release_authority_ref: "release-authority:offline".into(),
            release_policy_digest: d(8),
            release_policy_profile: "coordinator-release-policy-v1".into(),
            valid_from_ms: now_ms.saturating_sub(5_000),
            valid_until_ms: now_ms + 120_000,
        }
    }

    fn authenticated(
        now_ms: u64,
        manifest: CoordinatorReleaseManifest,
    ) -> QualifiedCoordinatorReleaseRequirement {
        let digest = manifest.manifest_digest().unwrap();
        let proof = VerifiedCoordinatorReleaseSignatureProof {
            protocol_version: SIGNATURE_PROOF_PROTOCOL.into(),
            manifest_digest: digest,
            manifest_profile: MANIFEST_PROFILE.into(),
            release_authority_ref: manifest.release_authority_ref.clone(),
            release_policy_digest: manifest.release_policy_digest,
            release_policy_profile: manifest.release_policy_profile.clone(),
            signing_key_id: "test-key".into(),
            signature_ref: "test-signature".into(),
            verifier_ref: "test-verifier".into(),
            verified_at_ms: now_ms,
            valid_until_ms: now_ms + 30_000,
        };
        qualify_coordinator_release(manifest, &proof, now_ms).unwrap()
    }

    fn record(
        digest: [u8; 32],
        status: CoordinatorReleaseStatus,
        effective_at_ms: u64,
    ) -> CoordinatorReleaseRegistryRecord {
        CoordinatorReleaseRegistryRecord {
            manifest_digest: digest,
            manifest_profile: MANIFEST_PROFILE.into(),
            status,
            status_effective_at_ms: effective_at_ms,
            status_ref: format!("status:{}", digest[0]),
        }
    }

    fn snapshot(
        now_ms: u64,
        generation: u64,
        previous: Option<[u8; 32]>,
        records: Vec<CoordinatorReleaseRegistryRecord>,
        root_version: u64,
    ) -> CoordinatorReleaseRegistrySnapshot {
        CoordinatorReleaseRegistrySnapshot {
            protocol_version: REGISTRY_PROTOCOL.into(),
            root_version,
            registry_id: "coordinator-release-registry".into(),
            release_authority_ref: "release-authority:offline".into(),
            release_policy_digest: d(8),
            release_policy_profile: "coordinator-release-policy-v1".into(),
            registry_generation: generation,
            previous_registry_head_digest: previous,
            records,
            valid_from_ms: now_ms.saturating_sub(1_000),
            valid_until_ms: now_ms + 20_000,
        }
    }

    fn advance(
        store: &TrustedCoordinatorReleaseRegistryStore,
        root: &CoordinatorReleaseRegistryTrustRoot,
        snapshot: CoordinatorReleaseRegistrySnapshot,
        signers: &[&Signer],
    ) -> TrustedRegistryStateSummary {
        let message = snapshot_signature_message(root, &snapshot).unwrap();
        let signatures: Vec<_> = signers
            .iter()
            .take(usize::from(root.registry_head_threshold))
            .map(|signer| signer.sign(&message))
            .collect();
        store.advance_snapshot(snapshot, &signatures).unwrap()
    }

    #[test]
    fn latest_complete_snapshot_survives_restart_and_qualifies_without_caller_snapshot() {
        let now_ms = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 1);
        let r2 = Signer::new("r2", 2);
        let h1 = Signer::new("h1", 11);
        let h2 = Signer::new("h2", 12);
        let root = root(now_ms, &[&r1, &r2], &[&h1, &h2], 1, 2, 2);
        let dir = temp_dir();
        let path = dir.join("registry-state.json");
        let store = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        bootstrap(&store, root.clone(), &[&r1, &r2]);

        let manifest = manifest(now_ms, 21);
        let digest = manifest.manifest_digest().unwrap();
        let first = advance(
            &store,
            &root,
            snapshot(
                now_ms,
                1,
                None,
                vec![record(
                    digest,
                    CoordinatorReleaseStatus::Active,
                    now_ms.saturating_sub(2_000),
                )],
                1,
            ),
            &[&h1, &h2],
        );
        assert_eq!(first.registry_generation, Some(1));
        drop(store);

        let restarted = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        let before = restarted.state_summary().unwrap().state_generation;
        let qualified = restarted
            .qualify_current_release(authenticated(now_ms, manifest.clone()))
            .unwrap();
        assert_eq!(qualified.release().manifest_digest(), digest);
        assert!(restarted.state_summary().unwrap().state_generation > before);
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn older_or_record_removing_snapshot_denies_after_restart() {
        let now_ms = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 21);
        let r2 = Signer::new("r2", 22);
        let h1 = Signer::new("h1", 31);
        let h2 = Signer::new("h2", 32);
        let root = root(now_ms, &[&r1, &r2], &[&h1, &h2], 1, 2, 2);
        let dir = temp_dir();
        let path = dir.join("registry-state.json");
        let store = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        bootstrap(&store, root.clone(), &[&r1, &r2]);

        let digest = d(41);
        let first = advance(
            &store,
            &root,
            snapshot(
                now_ms,
                1,
                None,
                vec![record(
                    digest,
                    CoordinatorReleaseStatus::Active,
                    now_ms.saturating_sub(3_000),
                )],
                1,
            ),
            &[&h1, &h2],
        );
        let second = advance(
            &store,
            &root,
            snapshot(
                now_ms,
                2,
                first.registry_head_digest,
                vec![record(
                    digest,
                    CoordinatorReleaseStatus::Withdrawn,
                    now_ms.saturating_sub(2_000),
                )],
                1,
            ),
            &[&h1, &h2],
        );
        drop(store);

        let restarted = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        let old = snapshot(
            now_ms,
            1,
            None,
            vec![record(
                digest,
                CoordinatorReleaseStatus::Active,
                now_ms.saturating_sub(3_000),
            )],
            1,
        );
        let message = snapshot_signature_message(&root, &old).unwrap();
        let signatures = vec![h1.sign(&message), h2.sign(&message)];
        assert!(matches!(
            restarted.advance_snapshot(old, &signatures),
            Err(TrustedRegistryStateError::SnapshotPredecessorMismatch)
        ));

        let removed = snapshot(now_ms, 3, second.registry_head_digest, vec![], 1);
        let message = snapshot_signature_message(&root, &removed).unwrap();
        let signatures = vec![h1.sign(&message), h2.sign(&message)];
        assert_eq!(
            restarted.advance_snapshot(removed, &signatures).unwrap_err(),
            TrustedRegistryStateError::RegistryRecordRemoved
        );
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn withdrawn_release_denies_from_state_owned_snapshot() {
        let now_ms = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 41);
        let r2 = Signer::new("r2", 42);
        let h1 = Signer::new("h1", 51);
        let h2 = Signer::new("h2", 52);
        let root = root(now_ms, &[&r1, &r2], &[&h1, &h2], 1, 2, 2);
        let dir = temp_dir();
        let path = dir.join("registry-state.json");
        let store = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        bootstrap(&store, root.clone(), &[&r1, &r2]);

        let manifest = manifest(now_ms, 61);
        let digest = manifest.manifest_digest().unwrap();
        advance(
            &store,
            &root,
            snapshot(
                now_ms,
                1,
                None,
                vec![record(
                    digest,
                    CoordinatorReleaseStatus::Withdrawn,
                    now_ms.saturating_sub(2_000),
                )],
                1,
            ),
            &[&h1, &h2],
        );
        assert_eq!(
            store
                .qualify_current_release(authenticated(now_ms, manifest))
                .unwrap_err(),
            TrustedRegistryStateError::ReleaseNotActive(CoordinatorReleaseStatus::Withdrawn)
        );
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn root_rotation_is_state_owned_thresholds_ratchet_and_old_snapshot_stops_authorizing() {
        let now_ms = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 61);
        let r2 = Signer::new("r2", 62);
        let r3 = Signer::new("r3", 63);
        let h1 = Signer::new("h1", 71);
        let h2 = Signer::new("h2", 72);
        let h3 = Signer::new("h3", 73);
        let old = root(
            now_ms,
            &[&r1, &r2, &r3],
            &[&h1, &h2, &h3],
            1,
            1,
            1,
        );
        let dir = temp_dir();
        let path = dir.join("registry-state.json");
        let store = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        bootstrap(&store, old.clone(), &[&r1]);

        let release_manifest = manifest(now_ms, 71);
        let release_digest = release_manifest.manifest_digest().unwrap();
        let first = advance(
            &store,
            &old,
            snapshot(
                now_ms,
                1,
                None,
                vec![record(
                    release_digest,
                    CoordinatorReleaseStatus::Active,
                    now_ms.saturating_sub(2_000),
                )],
                1,
            ),
            &[&h1],
        );

        let nr1 = Signer::new("nr1", 81);
        let nr2 = Signer::new("nr2", 82);
        let nr3 = Signer::new("nr3", 83);
        let nh1 = Signer::new("nh1", 91);
        let nh2 = Signer::new("nh2", 92);
        let nh3 = Signer::new("nh3", 93);
        let new = root(
            now_ms,
            &[&nr1, &nr2, &nr3],
            &[&nh1, &nh2, &nh3],
            2,
            2,
            2,
        );
        let message = root_rotation_signature_message(&old, &new).unwrap();
        let old_signatures = vec![r1.sign(&message)];
        let new_signatures = vec![nr1.sign(&message), nr2.sign(&message)];
        let summary = store
            .rotate_root(new.clone(), &old_signatures, &new_signatures)
            .unwrap();
        assert_eq!(summary.root_threshold_floor, 2);
        assert_eq!(summary.registry_head_threshold_floor, 2);
        assert_eq!(
            store
                .qualify_current_release(authenticated(now_ms, release_manifest.clone()))
                .unwrap_err(),
            TrustedRegistryStateError::SnapshotNotUnderCurrentRoot
        );

        let successor = snapshot(
            now_ms,
            2,
            first.registry_head_digest,
            vec![record(
                release_digest,
                CoordinatorReleaseStatus::Active,
                now_ms.saturating_sub(2_000),
            )],
            2,
        );
        advance(&store, &new, successor, &[&nh1, &nh2]);
        assert!(store
            .qualify_current_release(authenticated(now_ms, release_manifest))
            .is_ok());

        let weak = root(
            now_ms,
            &[&r1, &r2, &r3],
            &[&h1, &h2, &h3],
            3,
            1,
            1,
        );
        let message = root_rotation_signature_message(&new, &weak).unwrap();
        let old_signatures = vec![nr1.sign(&message), nr2.sign(&message)];
        let new_signatures = vec![r1.sign(&message)];
        assert_eq!(
            store
                .rotate_root(weak, &old_signatures, &new_signatures)
                .unwrap_err(),
            TrustedRegistryStateError::ThresholdFloorViolation
        );
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn trusted_clock_floor_detects_backward_time() {
        let now_ms = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 101);
        let r2 = Signer::new("r2", 102);
        let h1 = Signer::new("h1", 103);
        let h2 = Signer::new("h2", 104);
        let root = root(now_ms, &[&r1, &r2], &[&h1, &h2], 1, 2, 2);
        let dir = temp_dir();
        let path = dir.join("registry-state.json");
        let store = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        bootstrap(&store, root, &[&r1, &r2]);

        let mut state = store.load_state_unlocked().unwrap();
        state.last_trusted_time_ms = now_ms + 60_000;
        state.state_digest = state.compute_digest().unwrap();
        state.validate().unwrap();
        store.atomic_replace_state(&state).unwrap();
        assert!(matches!(
            store.qualify_current_release(authenticated(now_ms, manifest(now_ms, 115))),
            Err(TrustedRegistryStateError::ClockRollback { .. })
        ));
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn store_canonicalizes_parent_path_once() {
        use std::os::unix::fs::symlink;

        let real = temp_dir();
        let holder = temp_dir();
        let link = holder.join("registry-dir-link");
        symlink(&real, &link).unwrap();
        let store = TrustedCoordinatorReleaseRegistryStore::new(link.join("state.json")).unwrap();
        assert_eq!(
            store.state_path.parent().unwrap(),
            fs::canonicalize(&real).unwrap()
        );
        let _ = fs::remove_dir_all(holder);
        let _ = fs::remove_dir_all(real);
    }

    #[test]
    fn corrupted_state_digest_denies_on_load() {
        let now_ms = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 111);
        let r2 = Signer::new("r2", 112);
        let h1 = Signer::new("h1", 113);
        let h2 = Signer::new("h2", 114);
        let root = root(now_ms, &[&r1, &r2], &[&h1, &h2], 1, 2, 2);
        let dir = temp_dir();
        let path = dir.join("registry-state.json");
        let store = TrustedCoordinatorReleaseRegistryStore::new(&path).unwrap();
        bootstrap(&store, root, &[&r1, &r2]);

        let mut value: serde_json::Value =
            serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        value["last_trusted_time_ms"] = serde_json::Value::from(now_ms + 1);
        fs::write(&path, serde_json::to_vec(&value).unwrap()).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(STATE_FILE_MODE)).unwrap();
        assert_eq!(
            store.state_summary().unwrap_err(),
            TrustedRegistryStateError::StateDigestMismatch
        );
        let _ = fs::remove_dir_all(dir);
    }
}

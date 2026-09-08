// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Crash-durable local trusted state for coordinator release-key policy.
//!
//! Normal verification accepts candidate root/head/policy/signature data, but never
//! caller-supplied prior trusted state, prior root pin, prior head or prior generation.
//! Those are loaded from one configured owner-only path while an exclusive lock is
//! held. State is fsynced and atomically replaced before positive #307 authority can
//! escape.
//!
//! This protects against caller rollback, concurrent lost updates, torn writes and
//! restart loss under normal host/filesystem protection. It does not claim resistance
//! to restoring an entire older machine/filesystem image; that requires a later
//! hardware/enterprise monotonic anchor.

#![cfg_attr(not(unix), allow(dead_code))]

#[cfg(not(unix))]
compile_error!("coordinator release key-policy trusted state v0.1 requires Unix");

use ed25519_dalek::{Signature as EdSignature, VerifyingKey as EdVerifyingKey};
use ml_dsa::signature::Verifier as MlVerifier;
use ml_dsa::{
    EncodedSignature, EncodedVerifyingKey, KeyInit as _, MlDsa65,
    Signature as MlSignature, VerifyingKey as MlVerifyingKey,
};
use mycelix_authority_coordinator_release::CoordinatorReleaseManifest;
use mycelix_authority_coordinator_release_key_policy::{
    CoordinatorReleaseKeyPolicy, QualifiedCoordinatorReleaseManifestKeyPolicy,
    VerifiedCurrentCoordinatorReleaseKeyPolicyProof, CURRENT_POLICY_PROOF_PROTOCOL,
    MAX_CURRENTNESS_PROOF_REUSE_MS, POLICY_PROFILE,
    qualify_manifest_key_policy as qualify_manifest_key_policy_v01,
};
use mycelix_authority_coordinator_release_key_policy_root::{
    CoordinatorReleaseKeyPolicyHead, CoordinatorReleaseKeyPolicyRootPin,
    CoordinatorReleaseKeyPolicyTrustRoot, HybridPolicyTrustKey, HybridThresholdSignature,
    POLICY_HEAD_PROFILE, ROOT_PROFILE, THRESHOLD_SIGNATURE_PROFILE, bootstrap_policy_root,
    policy_head_signature_message, root_rotation_signature_message,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-release-key-policy-state-v0.1";
pub const STATE_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-state-v1-blake3-framed";
pub const HEAD_CHECKPOINT_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-head-checkpoint-v1-blake3-framed";
pub const VERIFIER_REF_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-state-verifier-ref-v1-blake3";
pub const STATE_FILE_MODE: u32 = 0o600;
pub const MAX_STATE_BYTES: usize = 4 * 1024 * 1024;

const DOMAIN_STATE: &[u8] = b"mycelix/authority/coordinator-release-key-policy-state/v1";
const DOMAIN_HEAD_CHECKPOINT: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-head-checkpoint/v1";
const DOMAIN_VERIFIER_REF: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-state-verifier-ref/v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedPolicyHeadCheckpoint {
    pub root_version: u64,
    pub policy_digest: [u8; 32],
    pub policy_profile: String,
    pub policy_generation: u64,
    pub head_digest: [u8; 32],
    pub head_profile: String,
    pub previous_head_digest: Option<[u8; 32]>,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl TrustedPolicyHeadCheckpoint {
    fn validate(&self) -> Result<(), TrustedPolicyStateError> {
        if self.root_version == 0 || self.policy_generation == 0 {
            return Err(TrustedPolicyStateError::InvalidHeadCheckpoint);
        }
        validate_digest(&self.policy_digest, "checkpoint policy digest")?;
        validate_digest(&self.head_digest, "checkpoint head digest")?;
        if self.policy_profile != POLICY_PROFILE || self.head_profile != POLICY_HEAD_PROFILE {
            return Err(TrustedPolicyStateError::InvalidHeadCheckpoint);
        }
        if let Some(previous) = self.previous_head_digest {
            validate_digest(&previous, "checkpoint predecessor digest")?;
        }
        if self.verified_at_ms == 0 || self.valid_until_ms <= self.verified_at_ms {
            return Err(TrustedPolicyStateError::InvalidHeadCheckpoint);
        }
        Ok(())
    }

    fn digest(&self) -> Result<[u8; 32], TrustedPolicyStateError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_HEAD_CHECKPOINT);
        frame_hash(&mut h, HEAD_CHECKPOINT_PROFILE.as_bytes());
        frame_hash(&mut h, &self.root_version.to_le_bytes());
        frame_hash(&mut h, &self.policy_digest);
        frame_hash(&mut h, self.policy_profile.as_bytes());
        frame_hash(&mut h, &self.policy_generation.to_le_bytes());
        frame_hash(&mut h, &self.head_digest);
        frame_hash(&mut h, self.head_profile.as_bytes());
        match self.previous_head_digest {
            Some(previous) => {
                frame_hash(&mut h, b"previous");
                frame_hash(&mut h, &previous);
            }
            None => frame_hash(&mut h, b"genesis"),
        }
        frame_hash(&mut h, &self.verified_at_ms.to_le_bytes());
        frame_hash(&mut h, &self.valid_until_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

/// Serialized checkpoint data. It is not itself a positive authority capability.
/// The live authority comes from loading it from the configured secure path under
/// the store's exclusive transaction lock.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedCoordinatorReleaseKeyPolicyState {
    pub protocol_version: String,
    pub state_profile: String,
    pub state_generation: u64,
    pub previous_state_digest: Option<[u8; 32]>,
    pub bootstrap_root_digest: [u8; 32],
    pub bootstrap_root_profile: String,
    pub current_root: CoordinatorReleaseKeyPolicyTrustRoot,
    pub current_root_digest: [u8; 32],
    pub root_threshold_floor: u16,
    pub policy_head_threshold_floor: u16,
    pub current_head: Option<TrustedPolicyHeadCheckpoint>,
    pub last_trusted_time_ms: u64,
    pub state_digest: [u8; 32],
}

impl TrustedCoordinatorReleaseKeyPolicyState {
    fn validate(&self) -> Result<(), TrustedPolicyStateError> {
        if self.protocol_version != PROTOCOL_VERSION || self.state_profile != STATE_PROFILE {
            return Err(TrustedPolicyStateError::WrongStateProtocol);
        }
        if self.state_generation == 0 {
            return Err(TrustedPolicyStateError::InvalidStateGeneration);
        }
        match (self.state_generation, self.previous_state_digest) {
            (1, None) => {}
            (1, Some(_)) | (_, None) => {
                return Err(TrustedPolicyStateError::InvalidPreviousStateDigest);
            }
            (_, Some(previous)) => validate_digest(&previous, "previous state digest")?,
        }
        validate_digest(&self.bootstrap_root_digest, "bootstrap root digest")?;
        if self.bootstrap_root_profile != ROOT_PROFILE {
            return Err(TrustedPolicyStateError::WrongRootProfile);
        }

        self.current_root
            .validate()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        let actual_root_digest = self
            .current_root
            .root_digest()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        if actual_root_digest != self.current_root_digest {
            return Err(TrustedPolicyStateError::CurrentRootDigestMismatch);
        }
        if self.current_root.root_version == 1
            && self.current_root_digest != self.bootstrap_root_digest
        {
            return Err(TrustedPolicyStateError::BootstrapRootMismatch);
        }
        if self.root_threshold_floor == 0
            || self.policy_head_threshold_floor == 0
            || self.current_root.root_threshold < self.root_threshold_floor
            || self.current_root.policy_head_threshold < self.policy_head_threshold_floor
        {
            return Err(TrustedPolicyStateError::ThresholdFloorViolation);
        }
        if self.last_trusted_time_ms == 0
            || self.last_trusted_time_ms >= self.current_root.valid_until_ms
        {
            return Err(TrustedPolicyStateError::InvalidTrustedClockFloor);
        }
        if let Some(head) = &self.current_head {
            head.validate()?;
            if head.root_version > self.current_root.root_version
                || head.verified_at_ms > self.last_trusted_time_ms
            {
                return Err(TrustedPolicyStateError::InvalidHeadCheckpoint);
            }
        }
        if self.compute_digest()? != self.state_digest {
            return Err(TrustedPolicyStateError::StateDigestMismatch);
        }
        Ok(())
    }

    fn compute_digest(&self) -> Result<[u8; 32], TrustedPolicyStateError> {
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_STATE);
        frame_hash(&mut h, PROTOCOL_VERSION.as_bytes());
        frame_hash(&mut h, STATE_PROFILE.as_bytes());
        frame_hash(&mut h, &self.state_generation.to_le_bytes());
        match self.previous_state_digest {
            Some(previous) => {
                frame_hash(&mut h, b"previous");
                frame_hash(&mut h, &previous);
            }
            None => frame_hash(&mut h, b"genesis"),
        }
        frame_hash(&mut h, &self.bootstrap_root_digest);
        frame_hash(&mut h, self.bootstrap_root_profile.as_bytes());
        frame_hash(&mut h, &self.current_root_digest);
        frame_hash(&mut h, ROOT_PROFILE.as_bytes());
        frame_hash(&mut h, &self.root_threshold_floor.to_le_bytes());
        frame_hash(&mut h, &self.policy_head_threshold_floor.to_le_bytes());
        match &self.current_head {
            Some(head) => {
                frame_hash(&mut h, b"head");
                frame_hash(&mut h, &head.digest()?);
                frame_hash(&mut h, HEAD_CHECKPOINT_PROFILE.as_bytes());
            }
            None => frame_hash(&mut h, b"no-head"),
        }
        frame_hash(&mut h, &self.last_trusted_time_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }

    fn advance(
        &self,
        current_root: CoordinatorReleaseKeyPolicyTrustRoot,
        current_head: Option<TrustedPolicyHeadCheckpoint>,
        trusted_time_ms: u64,
        root_threshold_floor: u16,
        policy_head_threshold_floor: u16,
    ) -> Result<Self, TrustedPolicyStateError> {
        if trusted_time_ms < self.last_trusted_time_ms {
            return Err(TrustedPolicyStateError::ClockRollback {
                stored_ms: self.last_trusted_time_ms,
                observed_ms: trusted_time_ms,
            });
        }
        if root_threshold_floor < self.root_threshold_floor
            || policy_head_threshold_floor < self.policy_head_threshold_floor
        {
            return Err(TrustedPolicyStateError::ThresholdFloorViolation);
        }
        let state_generation = self
            .state_generation
            .checked_add(1)
            .ok_or(TrustedPolicyStateError::StateGenerationOverflow)?;
        let current_root_digest = current_root
            .root_digest()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
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
            policy_head_threshold_floor,
            current_head,
            last_trusted_time_ms: trusted_time_ms,
            state_digest: [0; 32],
        };
        next.state_digest = next.compute_digest()?;
        next.validate()?;
        Ok(next)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrustedPolicyStateSummary {
    pub state_generation: u64,
    pub state_digest: [u8; 32],
    pub root_version: u64,
    pub root_digest: [u8; 32],
    pub root_threshold_floor: u16,
    pub policy_head_threshold_floor: u16,
    pub policy_generation: Option<u64>,
    pub policy_head_digest: Option<[u8; 32]>,
    pub last_trusted_time_ms: u64,
}

impl From<&TrustedCoordinatorReleaseKeyPolicyState> for TrustedPolicyStateSummary {
    fn from(state: &TrustedCoordinatorReleaseKeyPolicyState) -> Self {
        Self {
            state_generation: state.state_generation,
            state_digest: state.state_digest,
            root_version: state.current_root.root_version,
            root_digest: state.current_root_digest,
            root_threshold_floor: state.root_threshold_floor,
            policy_head_threshold_floor: state.policy_head_threshold_floor,
            policy_generation: state.current_head.as_ref().map(|head| head.policy_generation),
            policy_head_digest: state.current_head.as_ref().map(|head| head.head_digest),
            last_trusted_time_ms: state.last_trusted_time_ms,
        }
    }
}

/// Fixed-path trusted-state boundary. Per-request APIs do not accept a trusted-state
/// object, root pin, previous head, prior root or prior generation.
#[derive(Clone, Debug)]
pub struct TrustedCoordinatorReleaseKeyPolicyStore {
    state_path: PathBuf,
    lock_path: PathBuf,
}

impl TrustedCoordinatorReleaseKeyPolicyStore {
    pub fn new(state_path: impl Into<PathBuf>) -> Result<Self, TrustedPolicyStateError> {
        let state_path = state_path.into();
        let parent = state_path
            .parent()
            .ok_or(TrustedPolicyStateError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        let file_name = state_path
            .file_name()
            .ok_or(TrustedPolicyStateError::StatePathHasNoFileName)?
            .to_string_lossy();
        let lock_path = state_path.with_file_name(format!("{file_name}.lock"));
        Ok(Self {
            state_path,
            lock_path,
        })
    }

    /// First-use provisioning only. The pin must be independently supplied out of
    /// band. Existing state, including a symlink at the state path, is never replaced.
    pub fn bootstrap_from_out_of_band_pin(
        &self,
        root: CoordinatorReleaseKeyPolicyTrustRoot,
        pin: &CoordinatorReleaseKeyPolicyRootPin,
        root_signatures: &[HybridThresholdSignature],
    ) -> Result<TrustedPolicyStateSummary, TrustedPolicyStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        self.require_state_absent()?;
        let qualified = bootstrap_policy_root(root.clone(), pin, root_signatures)
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        let verified_at_ms = qualified.verified_at_ms();
        let current_root_digest = qualified.root_digest();
        let mut state = TrustedCoordinatorReleaseKeyPolicyState {
            protocol_version: PROTOCOL_VERSION.into(),
            state_profile: STATE_PROFILE.into(),
            state_generation: 1,
            previous_state_digest: None,
            bootstrap_root_digest: current_root_digest,
            bootstrap_root_profile: ROOT_PROFILE.into(),
            current_root: root.clone(),
            current_root_digest,
            root_threshold_floor: root.root_threshold,
            policy_head_threshold_floor: root.policy_head_threshold,
            current_head: None,
            last_trusted_time_ms: verified_at_ms,
            state_digest: [0; 32],
        };
        state.state_digest = state.compute_digest()?;
        state.validate()?;
        self.atomic_replace_state(&state)?;
        Ok((&state).into())
    }

    /// Rotate the state-owned root. The old root is never a caller parameter.
    /// Threshold floors ratchet upward when a new root strengthens them.
    pub fn rotate_root(
        &self,
        new_root: CoordinatorReleaseKeyPolicyTrustRoot,
        old_root_signatures: &[HybridThresholdSignature],
        new_root_signatures: &[HybridThresholdSignature],
    ) -> Result<TrustedPolicyStateSummary, TrustedPolicyStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        let started_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        let old_root = &state.current_root;
        old_root
            .validate()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        new_root
            .validate()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        if !root_live_at(old_root, started_at_ms) || !root_live_at(&new_root, started_at_ms) {
            return Err(TrustedPolicyStateError::RootNotLive);
        }
        if new_root.root_id != old_root.root_id
            || new_root.release_authority_ref != old_root.release_authority_ref
            || new_root.policy_id != old_root.policy_id
        {
            return Err(TrustedPolicyStateError::RootScopeChanged);
        }
        let expected_version = old_root
            .root_version
            .checked_add(1)
            .ok_or(TrustedPolicyStateError::RootVersionOverflow)?;
        if new_root.root_version != expected_version {
            return Err(TrustedPolicyStateError::InvalidRootRotationVersion);
        }
        if new_root.root_threshold < state.root_threshold_floor
            || new_root.policy_head_threshold < state.policy_head_threshold_floor
        {
            return Err(TrustedPolicyStateError::ThresholdFloorViolation);
        }

        let message = root_rotation_signature_message(old_root, &new_root)
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
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
            return Err(TrustedPolicyStateError::RootExpiredDuringVerification);
        }

        let ratcheted_root_floor = state.root_threshold_floor.max(new_root.root_threshold);
        let ratcheted_head_floor = state
            .policy_head_threshold_floor
            .max(new_root.policy_head_threshold);
        let next = state.advance(
            new_root,
            state.current_head.clone(),
            verified_at_ms,
            ratcheted_root_floor,
            ratcheted_head_floor,
        )?;
        self.atomic_replace_state(&next)?;
        Ok((&next).into())
    }

    /// Verify/revalidate the signed policy head against the state-owned root, persist
    /// the latest checkpoint, and only then construct #307 positive authority.
    pub fn qualify_manifest_key_policy(
        &self,
        policy: CoordinatorReleaseKeyPolicy,
        head: CoordinatorReleaseKeyPolicyHead,
        head_signatures: &[HybridThresholdSignature],
        manifest: &CoordinatorReleaseManifest,
    ) -> Result<QualifiedCoordinatorReleaseManifestKeyPolicy, TrustedPolicyStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        let started_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        let root = &state.current_root;
        if !root_live_at(root, started_at_ms) {
            return Err(TrustedPolicyStateError::RootNotLive);
        }

        policy
            .validate()
            .map_err(|error| TrustedPolicyStateError::KeyPolicy(error.to_string()))?;
        head.validate()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        manifest
            .validate()
            .map_err(|error| TrustedPolicyStateError::Manifest(error.to_string()))?;
        let policy_digest = policy
            .policy_digest()
            .map_err(|error| TrustedPolicyStateError::KeyPolicy(error.to_string()))?;
        let head_digest = head
            .head_digest()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;

        if policy.policy_id != root.policy_id
            || policy.release_authority_ref != root.release_authority_ref
        {
            return Err(TrustedPolicyStateError::PolicyOutsideRootScope);
        }
        if manifest.release_authority_ref != policy.release_authority_ref
            || manifest.release_policy_profile != POLICY_PROFILE
            || manifest.release_policy_digest != policy_digest
        {
            return Err(TrustedPolicyStateError::ManifestPolicyMismatch);
        }
        if head.root_version != root.root_version
            || head.policy_digest != policy_digest
            || head.policy_profile != POLICY_PROFILE
            || head.policy_generation != policy.policy_generation
        {
            return Err(TrustedPolicyStateError::HeadPolicyBindingMismatch);
        }
        require_live_window(
            policy.valid_from_ms,
            policy.valid_until_ms,
            started_at_ms,
            TrustedPolicyStateError::PolicyNotLive,
        )?;
        require_live_window(
            head.valid_from_ms,
            head.valid_until_ms,
            started_at_ms,
            TrustedPolicyStateError::HeadNotLive,
        )?;
        self.verify_lineage(&state, &head, head_digest, policy_digest)?;

        let message = policy_head_signature_message(root, &head)
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        verify_exact_threshold(
            &root.policy_head_keys,
            root.policy_head_threshold,
            head_signatures,
            &message,
            started_at_ms,
        )?;
        let verified_at_ms = checked_time_after_floor(state.last_trusted_time_ms)?;
        ensure_signers_live(&root.policy_head_keys, head_signatures, verified_at_ms)?;
        if !root_live_at(root, verified_at_ms) {
            return Err(TrustedPolicyStateError::RootExpiredDuringVerification);
        }
        require_live_window(
            policy.valid_from_ms,
            policy.valid_until_ms,
            verified_at_ms,
            TrustedPolicyStateError::PolicyExpiredDuringVerification,
        )?;
        require_live_window(
            head.valid_from_ms,
            head.valid_until_ms,
            verified_at_ms,
            TrustedPolicyStateError::HeadExpiredDuringVerification,
        )?;

        let valid_until_ms = root
            .valid_until_ms
            .min(policy.valid_until_ms)
            .min(head.valid_until_ms)
            .min(signer_horizon(&root.policy_head_keys, head_signatures)?);
        if valid_until_ms <= verified_at_ms
            || valid_until_ms - verified_at_ms > MAX_CURRENTNESS_PROOF_REUSE_MS
        {
            return Err(TrustedPolicyStateError::InvalidCurrentnessWindow);
        }
        let checkpoint = TrustedPolicyHeadCheckpoint {
            root_version: head.root_version,
            policy_digest,
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: head.policy_generation,
            head_digest,
            head_profile: POLICY_HEAD_PROFILE.into(),
            previous_head_digest: head.previous_head_digest,
            verified_at_ms,
            valid_until_ms,
        };
        checkpoint.validate()?;
        let next = state.advance(
            root.clone(),
            Some(checkpoint),
            verified_at_ms,
            state.root_threshold_floor,
            state.policy_head_threshold_floor,
        )?;

        // Persistence MUST happen before a positive #307 capability is constructed.
        self.atomic_replace_state(&next)?;

        let receipt = VerifiedCurrentCoordinatorReleaseKeyPolicyProof {
            protocol_version: CURRENT_POLICY_PROOF_PROTOCOL.into(),
            policy_digest,
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: head.policy_generation,
            source_ref: format!("trusted-policy-head-blake3:{}", encode_hex(&head_digest)),
            verifier_ref: verifier_ref(&next),
            verified_at_ms,
            valid_until_ms,
        };
        qualify_manifest_key_policy_v01(manifest, policy, &receipt, verified_at_ms)
            .map_err(|error| TrustedPolicyStateError::KeyPolicy(error.to_string()))
    }

    /// Diagnostics only; never a positive authority result.
    pub fn state_summary(&self) -> Result<TrustedPolicyStateSummary, TrustedPolicyStateError> {
        let _lock = self.acquire_exclusive_lock()?;
        let state = self.load_state_unlocked()?;
        Ok((&state).into())
    }

    fn verify_lineage(
        &self,
        state: &TrustedCoordinatorReleaseKeyPolicyState,
        head: &CoordinatorReleaseKeyPolicyHead,
        head_digest: [u8; 32],
        policy_digest: [u8; 32],
    ) -> Result<(), TrustedPolicyStateError> {
        match &state.current_head {
            None => {
                if state.current_root.root_version != 1
                    || head.policy_generation != 1
                    || head.previous_head_digest.is_some()
                {
                    return Err(TrustedPolicyStateError::InvalidHeadGenesis);
                }
            }
            Some(current) if current.head_digest == head_digest => {
                if current.root_version != head.root_version
                    || current.policy_generation != head.policy_generation
                    || current.policy_digest != policy_digest
                {
                    return Err(TrustedPolicyStateError::HeadCheckpointMismatch);
                }
            }
            Some(current) => {
                let expected_generation = current
                    .policy_generation
                    .checked_add(1)
                    .ok_or(TrustedPolicyStateError::PolicyGenerationOverflow)?;
                if head.policy_generation != expected_generation
                    || head.previous_head_digest != Some(current.head_digest)
                    || head.root_version < current.root_version
                {
                    return Err(TrustedPolicyStateError::HeadPredecessorMismatch);
                }
            }
        }
        Ok(())
    }

    fn acquire_exclusive_lock(&self) -> Result<File, TrustedPolicyStateError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(TrustedPolicyStateError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.lock_path)?;
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(STATE_FILE_MODE)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.lock_path)
            .map_err(io_error("open trusted-state lock"))?;
        require_secure_regular_file(&lock, "trusted-state lock")?;
        lock.lock()
            .map_err(io_error("acquire trusted-state exclusive lock"))?;
        Ok(lock)
    }

    fn require_state_absent(&self) -> Result<(), TrustedPolicyStateError> {
        match fs::symlink_metadata(&self.state_path) {
            Ok(_) => Err(TrustedPolicyStateError::AlreadyInitialized),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(TrustedPolicyStateError::Io {
                operation: "inspect bootstrap state path",
                message: error.to_string(),
            }),
        }
    }

    fn load_state_unlocked(
        &self,
    ) -> Result<TrustedCoordinatorReleaseKeyPolicyState, TrustedPolicyStateError> {
        reject_symlink_if_present(&self.state_path)?;
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&self.state_path)
            .map_err(io_error("open trusted state"))?;
        require_secure_regular_file(&file, "trusted state")?;
        let len = file
            .metadata()
            .map_err(io_error("stat trusted state"))?
            .len();
        if len == 0 || len > MAX_STATE_BYTES as u64 {
            return Err(TrustedPolicyStateError::InvalidStateSize(len));
        }
        let mut bytes = Vec::with_capacity(len as usize);
        file.take((MAX_STATE_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(io_error("read trusted state"))?;
        if bytes.len() > MAX_STATE_BYTES {
            return Err(TrustedPolicyStateError::InvalidStateSize(bytes.len() as u64));
        }
        let state: TrustedCoordinatorReleaseKeyPolicyState = serde_json::from_slice(&bytes)
            .map_err(|error| TrustedPolicyStateError::Json(error.to_string()))?;
        state.validate()?;
        Ok(state)
    }

    fn atomic_replace_state(
        &self,
        state: &TrustedCoordinatorReleaseKeyPolicyState,
    ) -> Result<(), TrustedPolicyStateError> {
        state.validate()?;
        let parent = self
            .state_path
            .parent()
            .ok_or(TrustedPolicyStateError::StatePathHasNoParent)?;
        require_secure_parent_directory(parent)?;
        reject_symlink_if_present(&self.state_path)?;
        let bytes = serde_json::to_vec(state)
            .map_err(|error| TrustedPolicyStateError::Json(error.to_string()))?;
        if bytes.len() + 1 > MAX_STATE_BYTES {
            return Err(TrustedPolicyStateError::InvalidStateSize((bytes.len() + 1) as u64));
        }

        let tmp_path = self.unique_temp_path(state.state_generation)?;
        let result = (|| {
            let mut file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .mode(STATE_FILE_MODE)
                .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
                .open(&tmp_path)
                .map_err(io_error("create trusted-state temp"))?;
            require_secure_regular_file(&file, "trusted-state temp")?;
            file.write_all(&bytes)
                .map_err(io_error("write trusted-state temp"))?;
            file.write_all(b"\n")
                .map_err(io_error("finish trusted-state temp"))?;
            file.sync_all()
                .map_err(io_error("fsync trusted-state temp"))?;
            drop(file);
            fs::rename(&tmp_path, &self.state_path)
                .map_err(io_error("atomically replace trusted state"))?;
            let directory = OpenOptions::new()
                .read(true)
                .custom_flags(libc::O_DIRECTORY | libc::O_CLOEXEC)
                .open(parent)
                .map_err(io_error("open trusted-state directory"))?;
            directory
                .sync_all()
                .map_err(io_error("fsync trusted-state directory"))?;
            Ok(())
        })();
        if result.is_err() {
            let _ = fs::remove_file(&tmp_path);
        }
        result
    }

    fn unique_temp_path(&self, generation: u64) -> Result<PathBuf, TrustedPolicyStateError> {
        let parent = self
            .state_path
            .parent()
            .ok_or(TrustedPolicyStateError::StatePathHasNoParent)?;
        let base = self
            .state_path
            .file_name()
            .ok_or(TrustedPolicyStateError::StatePathHasNoFileName)?
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
        Err(TrustedPolicyStateError::TemporaryPathExhausted)
    }
}

fn root_live_at(root: &CoordinatorReleaseKeyPolicyTrustRoot, now_ms: u64) -> bool {
    root.valid_from_ms <= now_ms && now_ms < root.valid_until_ms
}

fn require_live_window(
    valid_from_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
    error: TrustedPolicyStateError,
) -> Result<(), TrustedPolicyStateError> {
    if valid_from_ms > now_ms || valid_until_ms <= now_ms || valid_until_ms <= valid_from_ms {
        Err(error)
    } else {
        Ok(())
    }
}

fn verify_exact_threshold(
    keys: &[HybridPolicyTrustKey],
    threshold: u16,
    signatures: &[HybridThresholdSignature],
    message: &[u8],
    now_ms: u64,
) -> Result<(), TrustedPolicyStateError> {
    if signatures.len() != usize::from(threshold) {
        return Err(TrustedPolicyStateError::ThresholdSignatureCountMismatch);
    }
    let mut signers = BTreeSet::new();
    for signature in signatures {
        signature
            .validate()
            .map_err(|error| TrustedPolicyStateError::Root(error.to_string()))?;
        if signature.signature_profile != THRESHOLD_SIGNATURE_PROFILE {
            return Err(TrustedPolicyStateError::WrongThresholdSignatureProfile);
        }
        if !signers.insert(signature.key_id.clone()) {
            return Err(TrustedPolicyStateError::DuplicateThresholdSigner);
        }
        let key = keys
            .iter()
            .find(|candidate| candidate.key_id == signature.key_id)
            .ok_or(TrustedPolicyStateError::UnauthorizedThresholdSigner)?;
        if !trust_key_live_at(key, now_ms) {
            return Err(TrustedPolicyStateError::ThresholdSignerNotLive);
        }
        verify_hybrid(key, message, signature)?;
    }
    Ok(())
}

fn ensure_signers_live(
    keys: &[HybridPolicyTrustKey],
    signatures: &[HybridThresholdSignature],
    now_ms: u64,
) -> Result<(), TrustedPolicyStateError> {
    for signature in signatures {
        let key = keys
            .iter()
            .find(|candidate| candidate.key_id == signature.key_id)
            .ok_or(TrustedPolicyStateError::UnauthorizedThresholdSigner)?;
        if !trust_key_live_at(key, now_ms) {
            return Err(TrustedPolicyStateError::ThresholdSignerExpiredDuringVerification);
        }
    }
    Ok(())
}

fn signer_horizon(
    keys: &[HybridPolicyTrustKey],
    signatures: &[HybridThresholdSignature],
) -> Result<u64, TrustedPolicyStateError> {
    signatures
        .iter()
        .map(|signature| {
            keys.iter()
                .find(|candidate| candidate.key_id == signature.key_id)
                .map(|key| key.valid_until_ms)
                .ok_or(TrustedPolicyStateError::UnauthorizedThresholdSigner)
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .min()
        .ok_or(TrustedPolicyStateError::ThresholdSignatureCountMismatch)
}

fn trust_key_live_at(key: &HybridPolicyTrustKey, now_ms: u64) -> bool {
    key.valid_from_ms <= now_ms && now_ms < key.valid_until_ms
}

fn verify_hybrid(
    key: &HybridPolicyTrustKey,
    message: &[u8],
    signature: &HybridThresholdSignature,
) -> Result<(), TrustedPolicyStateError> {
    let ed_key: [u8; 32] = key
        .ed25519_public_key
        .as_slice()
        .try_into()
        .map_err(|_| TrustedPolicyStateError::InvalidEd25519PublicKey)?;
    let ed_signature: [u8; 64] = signature
        .ed25519_signature
        .as_slice()
        .try_into()
        .map_err(|_| TrustedPolicyStateError::InvalidEd25519Signature)?;
    let ed_verifier = EdVerifyingKey::from_bytes(&ed_key)
        .map_err(|_| TrustedPolicyStateError::InvalidEd25519PublicKey)?;
    ed_verifier
        .verify_strict(message, &EdSignature::from_bytes(&ed_signature))
        .map_err(|_| TrustedPolicyStateError::Ed25519VerificationFailed)?;

    let encoded_key = EncodedVerifyingKey::<MlDsa65>::try_from(key.ml_dsa_65_public_key.as_slice())
        .map_err(|_| TrustedPolicyStateError::InvalidMlDsa65PublicKey)?;
    let ml_verifier = MlVerifyingKey::<MlDsa65>::decode(&encoded_key);
    let encoded_signature =
        EncodedSignature::<MlDsa65>::try_from(signature.ml_dsa_65_signature.as_slice())
            .map_err(|_| TrustedPolicyStateError::InvalidMlDsa65Signature)?;
    let ml_signature = MlSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or(TrustedPolicyStateError::InvalidMlDsa65Signature)?;
    ml_verifier
        .verify(message, &ml_signature)
        .map_err(|_| TrustedPolicyStateError::MlDsa65VerificationFailed)
}

fn verifier_ref(state: &TrustedCoordinatorReleaseKeyPolicyState) -> String {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_VERIFIER_REF);
    frame_hash(&mut h, VERIFIER_REF_PROFILE.as_bytes());
    frame_hash(&mut h, &state.state_digest);
    format!(
        "coordinator-release-key-policy-state-blake3:{}",
        encode_hex(h.finalize().as_bytes())
    )
}

fn checked_time_after_floor(floor_ms: u64) -> Result<u64, TrustedPolicyStateError> {
    let now_ms = system_time_ms()?;
    if now_ms < floor_ms {
        Err(TrustedPolicyStateError::ClockRollback {
            stored_ms: floor_ms,
            observed_ms: now_ms,
        })
    } else {
        Ok(now_ms)
    }
}

fn system_time_ms() -> Result<u64, TrustedPolicyStateError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| TrustedPolicyStateError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| TrustedPolicyStateError::ClockOverflow)
}

fn reject_symlink_if_present(path: &Path) -> Result<(), TrustedPolicyStateError> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            Err(TrustedPolicyStateError::SymlinkNotAllowed(path.to_path_buf()))
        }
        Ok(_) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(TrustedPolicyStateError::Io {
            operation: "inspect trusted-state path",
            message: error.to_string(),
        }),
    }
}

fn require_secure_parent_directory(path: &Path) -> Result<(), TrustedPolicyStateError> {
    let metadata = fs::symlink_metadata(path).map_err(io_error("stat trusted-state directory"))?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(TrustedPolicyStateError::InsecureParentDirectory);
    }
    if metadata.uid() != effective_uid() {
        return Err(TrustedPolicyStateError::WrongOwner("trusted-state directory"));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode & 0o077 != 0 || mode & 0o200 == 0 {
        return Err(TrustedPolicyStateError::InsecureDirectoryPermissions(mode));
    }
    Ok(())
}

fn require_secure_regular_file(
    file: &File,
    label: &'static str,
) -> Result<(), TrustedPolicyStateError> {
    let metadata = file.metadata().map_err(io_error("stat trusted-state file"))?;
    if !metadata.is_file() {
        return Err(TrustedPolicyStateError::NotRegularFile(label));
    }
    if metadata.uid() != effective_uid() {
        return Err(TrustedPolicyStateError::WrongOwner(label));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != STATE_FILE_MODE {
        return Err(TrustedPolicyStateError::InsecureFilePermissions { label, mode });
    }
    Ok(())
}

fn effective_uid() -> u32 {
    // SAFETY: geteuid takes no arguments and does not access caller memory.
    unsafe { libc::geteuid() }
}

fn validate_digest(value: &[u8; 32], field: &'static str) -> Result<(), TrustedPolicyStateError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(TrustedPolicyStateError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn frame_hash(hasher: &mut blake3::Hasher, bytes: &[u8]) {
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

fn io_error(operation: &'static str) -> impl FnOnce(std::io::Error) -> TrustedPolicyStateError {
    move |error| TrustedPolicyStateError::Io {
        operation,
        message: error.to_string(),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TrustedPolicyStateError {
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
    InvalidHeadCheckpoint,
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
    Root(String),
    KeyPolicy(String),
    Manifest(String),
    RootNotLive,
    RootExpiredDuringVerification,
    RootScopeChanged,
    RootVersionOverflow,
    InvalidRootRotationVersion,
    PolicyOutsideRootScope,
    ManifestPolicyMismatch,
    HeadPolicyBindingMismatch,
    PolicyNotLive,
    HeadNotLive,
    PolicyExpiredDuringVerification,
    HeadExpiredDuringVerification,
    InvalidHeadGenesis,
    HeadCheckpointMismatch,
    PolicyGenerationOverflow,
    HeadPredecessorMismatch,
    InvalidCurrentnessWindow,
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
    Io { operation: &'static str, message: String },
}

impl fmt::Display for TrustedPolicyStateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for TrustedPolicyStateError {}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as EdSigner, SigningKey as EdSigningKey};
    use ml_dsa::signature::{Keypair as _, Signer as MlSigner};
    use ml_dsa::{Generate as _, KeyExport as _, SigningKey as MlSigningKey};
    use mycelix_authority_coordinator_deployment::CoordinatorCodeIdentity;
    use mycelix_authority_coordinator_release::{
        CoordinatorReleaseManifest, PROTOCOL_VERSION as RELEASE_PROTOCOL,
    };
    use mycelix_authority_coordinator_release_key_policy::{
        AuthorizedHybridReleaseKey, PROTOCOL_VERSION as KEY_POLICY_PROTOCOL,
    };
    use mycelix_authority_coordinator_release_key_policy_root::{
        PROTOCOL_VERSION as ROOT_PROTOCOL, ROOT_PROFILE, root_bootstrap_signature_message,
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

        fn key(&self, now: u64) -> HybridPolicyTrustKey {
            HybridPolicyTrustKey {
                key_id: self.id.clone(),
                ed25519_public_key: self.ed.verifying_key().to_bytes().to_vec(),
                ml_dsa_65_public_key: self.ml.verifying_key().encode().as_slice().to_vec(),
                valid_from_ms: now.saturating_sub(5_000),
                valid_until_ms: now + 180_000,
            }
        }

        fn sign(&self, message: &[u8]) -> HybridThresholdSignature {
            HybridThresholdSignature {
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

    fn dir() -> PathBuf {
        let id = COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("mycelix-policy-state-{}-{id}", std::process::id()));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir(&path).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o700)).unwrap();
        path
    }

    fn d(byte: u8) -> [u8; 32] { [byte; 32] }
    fn h(byte: u8) -> Vec<u8> { vec![byte; 39] }

    fn root(
        now: u64,
        version: u64,
        roots: &[&Signer],
        policies: &[&Signer],
        root_threshold: u16,
        policy_threshold: u16,
    ) -> CoordinatorReleaseKeyPolicyTrustRoot {
        CoordinatorReleaseKeyPolicyTrustRoot {
            protocol_version: ROOT_PROTOCOL.into(),
            root_id: "coordinator-release-policy-root".into(),
            root_version: version,
            release_authority_ref: "release-authority:offline".into(),
            policy_id: "coordinator-release-keys".into(),
            root_keys: roots.iter().map(|s| s.key(now)).collect(),
            root_threshold,
            policy_head_keys: policies.iter().map(|s| s.key(now)).collect(),
            policy_head_threshold: policy_threshold,
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 180_000,
        }
    }

    fn policy(now: u64, generation: u64) -> CoordinatorReleaseKeyPolicy {
        CoordinatorReleaseKeyPolicy {
            protocol_version: KEY_POLICY_PROTOCOL.into(),
            policy_id: "coordinator-release-keys".into(),
            policy_generation: generation,
            release_authority_ref: "release-authority:offline".into(),
            authorized_keys: vec![AuthorizedHybridReleaseKey {
                key_id: "release-key-1".into(),
                ed25519_public_key: vec![7; 32],
                ml_dsa_65_public_key: vec![8; 1952],
                valid_from_ms: now.saturating_sub(5_000),
                valid_until_ms: now + 180_000,
            }],
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 180_000,
        }
    }

    fn head(
        root_version: u64,
        policy: &CoordinatorReleaseKeyPolicy,
        previous: Option<[u8; 32]>,
        now: u64,
    ) -> CoordinatorReleaseKeyPolicyHead {
        CoordinatorReleaseKeyPolicyHead {
            protocol_version: ROOT_PROTOCOL.into(),
            root_version,
            policy_digest: policy.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: policy.policy_generation,
            previous_head_digest: previous,
            valid_from_ms: now.saturating_sub(1_000),
            valid_until_ms: now + 20_000,
        }
    }

    fn manifest(policy: &CoordinatorReleaseKeyPolicy, now: u64) -> CoordinatorReleaseManifest {
        CoordinatorReleaseManifest {
            protocol_version: RELEASE_PROTOCOL.into(),
            release_id: "release-1".into(),
            release_version: 1,
            dna_hash_raw_39: h(9),
            coordinators: vec![CoordinatorCodeIdentity {
                zome_name: "authority_current_freshness_verifier".into(),
                wasm_hash_raw_39: h(1),
            }],
            dna_bundle_digest: d(3),
            source_tree_digest: d(4),
            lockfile_digest: d(5),
            toolchain_digest: d(6),
            build_recipe_digest: d(7),
            sbom_digest: d(8),
            source_ref: "git:tree:1".into(),
            build_ref: "nix:build:1".into(),
            release_authority_ref: policy.release_authority_ref.clone(),
            release_policy_digest: policy.policy_digest().unwrap(),
            release_policy_profile: POLICY_PROFILE.into(),
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 180_000,
        }
    }

    fn bootstrap(
        path: &Path,
        now: u64,
        roots: &[&Signer],
        policies: &[&Signer],
        root_threshold: u16,
        policy_threshold: u16,
    ) -> TrustedCoordinatorReleaseKeyPolicyStore {
        let store = TrustedCoordinatorReleaseKeyPolicyStore::new(path).unwrap();
        let candidate = root(now, 1, roots, policies, root_threshold, policy_threshold);
        let pin = CoordinatorReleaseKeyPolicyRootPin {
            root_digest: candidate.root_digest().unwrap(),
            root_profile: ROOT_PROFILE.into(),
        };
        let message = root_bootstrap_signature_message(&candidate).unwrap();
        let signatures: Vec<_> = roots
            .iter()
            .take(usize::from(root_threshold))
            .map(|signer| signer.sign(&message))
            .collect();
        store
            .bootstrap_from_out_of_band_pin(candidate, &pin, &signatures)
            .unwrap();
        store
    }

    #[test]
    fn bootstrap_is_single_use_and_owner_only() {
        let td = dir();
        let path = td.join("state.json");
        let now = system_time_ms().unwrap();
        let r = Signer::new("r1", 1);
        let p = Signer::new("p1", 2);
        let store = bootstrap(&path, now, &[&r], &[&p], 1, 1);
        assert_eq!(store.state_summary().unwrap().state_generation, 1);
        assert_eq!(fs::metadata(&path).unwrap().permissions().mode() & 0o777, 0o600);
        let candidate = root(now, 1, &[&r], &[&p], 1, 1);
        let pin = CoordinatorReleaseKeyPolicyRootPin {
            root_digest: candidate.root_digest().unwrap(),
            root_profile: ROOT_PROFILE.into(),
        };
        let msg = root_bootstrap_signature_message(&candidate).unwrap();
        assert_eq!(
            store
                .bootstrap_from_out_of_band_pin(candidate, &pin, &[r.sign(&msg)])
                .unwrap_err(),
            TrustedPolicyStateError::AlreadyInitialized
        );
        fs::remove_dir_all(td).unwrap();
    }

    #[test]
    fn restart_revalidation_and_successor_use_persisted_latest_head() {
        let td = dir();
        let path = td.join("state.json");
        let now = system_time_ms().unwrap();
        let r = Signer::new("r1", 11);
        let p = Signer::new("p1", 12);
        let store = bootstrap(&path, now, &[&r], &[&p], 1, 1);
        let root1 = root(now, 1, &[&r], &[&p], 1, 1);

        let p1 = policy(now, 1);
        let h1 = head(1, &p1, None, now);
        let m1 = policy_head_signature_message(&root1, &h1).unwrap();
        store
            .qualify_manifest_key_policy(p1.clone(), h1.clone(), &[p.sign(&m1)], &manifest(&p1, now))
            .unwrap();
        let first = store.state_summary().unwrap();

        let restarted = TrustedCoordinatorReleaseKeyPolicyStore::new(&path).unwrap();
        restarted
            .qualify_manifest_key_policy(p1.clone(), h1.clone(), &[p.sign(&m1)], &manifest(&p1, now))
            .unwrap();
        assert!(restarted.state_summary().unwrap().state_generation > first.state_generation);

        let p2 = policy(now, 2);
        let h2 = head(1, &p2, Some(h1.head_digest().unwrap()), now);
        let m2 = policy_head_signature_message(&root1, &h2).unwrap();
        restarted
            .qualify_manifest_key_policy(p2.clone(), h2, &[p.sign(&m2)], &manifest(&p2, now))
            .unwrap();
        assert_eq!(restarted.state_summary().unwrap().policy_generation, Some(2));

        assert_eq!(
            restarted
                .qualify_manifest_key_policy(p1.clone(), h1, &[p.sign(&m1)], &manifest(&p1, now))
                .unwrap_err(),
            TrustedPolicyStateError::HeadPredecessorMismatch
        );
        fs::remove_dir_all(td).unwrap();
    }

    #[test]
    fn root_rotation_ratchets_threshold_floors_and_cannot_reset_generation() {
        let td = dir();
        let path = td.join("state.json");
        let now = system_time_ms().unwrap();
        let or1 = Signer::new("or1", 21);
        let op1 = Signer::new("op1", 22);
        let store = bootstrap(&path, now, &[&or1], &[&op1], 1, 1);
        let old_root = root(now, 1, &[&or1], &[&op1], 1, 1);
        let p1 = policy(now, 1);
        let h1 = head(1, &p1, None, now);
        let hm1 = policy_head_signature_message(&old_root, &h1).unwrap();
        store
            .qualify_manifest_key_policy(p1.clone(), h1.clone(), &[op1.sign(&hm1)], &manifest(&p1, now))
            .unwrap();

        let nr1 = Signer::new("nr1", 31);
        let nr2 = Signer::new("nr2", 32);
        let np1 = Signer::new("np1", 33);
        let np2 = Signer::new("np2", 34);
        let new_root = root(now, 2, &[&nr1, &nr2], &[&np1, &np2], 2, 2);
        let rm = root_rotation_signature_message(&old_root, &new_root).unwrap();
        store
            .rotate_root(
                new_root.clone(),
                &[or1.sign(&rm)],
                &[nr1.sign(&rm), nr2.sign(&rm)],
            )
            .unwrap();
        let summary = store.state_summary().unwrap();
        assert_eq!(summary.root_threshold_floor, 2);
        assert_eq!(summary.policy_head_threshold_floor, 2);

        let reset = policy(now, 1);
        let reset_head = head(2, &reset, None, now);
        let reset_msg = policy_head_signature_message(&new_root, &reset_head).unwrap();
        assert_eq!(
            store
                .qualify_manifest_key_policy(
                    reset.clone(),
                    reset_head,
                    &[np1.sign(&reset_msg), np2.sign(&reset_msg)],
                    &manifest(&reset, now),
                )
                .unwrap_err(),
            TrustedPolicyStateError::HeadPredecessorMismatch
        );

        let weak_root = root(now, 3, &[&nr1], &[&np1], 1, 1);
        let weak_msg = root_rotation_signature_message(&new_root, &weak_root).unwrap();
        assert_eq!(
            store
                .rotate_root(
                    weak_root,
                    &[nr1.sign(&weak_msg), nr2.sign(&weak_msg)],
                    &[nr1.sign(&weak_msg)],
                )
                .unwrap_err(),
            TrustedPolicyStateError::ThresholdFloorViolation
        );
        fs::remove_dir_all(td).unwrap();
    }

    #[test]
    fn filesystem_and_clock_boundaries_fail_closed() {
        use std::os::unix::fs::symlink;

        let td = dir();
        let path = td.join("state.json");
        let now = system_time_ms().unwrap();
        let r = Signer::new("r1", 41);
        let p = Signer::new("p1", 42);
        let store = bootstrap(&path, now, &[&r], &[&p], 1, 1);

        fs::set_permissions(&path, fs::Permissions::from_mode(0o644)).unwrap();
        assert!(matches!(
            store.state_summary().unwrap_err(),
            TrustedPolicyStateError::InsecureFilePermissions { .. }
        ));
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        let link = td.join("state-link.json");
        symlink(&path, &link).unwrap();
        assert!(matches!(
            TrustedCoordinatorReleaseKeyPolicyStore::new(&link)
                .unwrap()
                .state_summary()
                .unwrap_err(),
            TrustedPolicyStateError::SymlinkNotAllowed(_)
        ));

        // Fixture changes only the locally trusted clock floor and self digest. It
        // deliberately leaves root-v1 metadata unchanged so bootstrap identity still
        // validates and the failure reaches the clock theorem.
        let lock = store.acquire_exclusive_lock().unwrap();
        let mut state = store.load_state_unlocked().unwrap();
        state.last_trusted_time_ms = system_time_ms().unwrap() + 60_000;
        state.state_digest = state.compute_digest().unwrap();
        store.atomic_replace_state(&state).unwrap();
        drop(lock);

        let p1 = policy(now, 1);
        let h1 = head(1, &p1, None, now);
        let root1 = root(now, 1, &[&r], &[&p], 1, 1);
        let message = policy_head_signature_message(&root1, &h1).unwrap();
        assert!(matches!(
            store
                .qualify_manifest_key_policy(p1.clone(), h1, &[p.sign(&message)], &manifest(&p1, now))
                .unwrap_err(),
            TrustedPolicyStateError::ClockRollback { .. }
        ));
        fs::remove_dir_all(td).unwrap();
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Offline-rooted currentness for coordinator-release signing-key policy.
//!
//! Release signing keys do not authorize this root. An out-of-band root pin
//! authenticates an offline hybrid-threshold root, which authorizes a separate
//! hybrid-threshold policy-head role. Only a short-lived, monotone policy head can
//! privately construct #307 currentness evidence and yield a non-deserializable
//! manifest/key-policy qualification.
//!
//! Root-pin provenance and durable monotonic-state persistence are deliberately
//! outside this crate and remain provisioning blockers.

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
    POLICY_PROFILE, qualify_manifest_key_policy as qualify_manifest_key_policy_v01,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-release-key-policy-root-v0.1";
pub const ROOT_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-root-v1-blake3-framed";
pub const ROOT_PIN_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-root-pin-v1-blake3";
pub const TRUST_KEY_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-trust-key-v1-ed25519-32-ml-dsa-65-1952";
pub const THRESHOLD_SIGNATURE_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-threshold-signature-v1-ed25519-64-ml-dsa-65-3309";
pub const ROOT_ROTATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-root-rotation-v1-blake3-framed";
pub const POLICY_HEAD_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-head-v1-blake3-framed";
pub const ROOT_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-root-qualification-v1-blake3-framed";
pub const POLICY_HEAD_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-head-qualification-v1-blake3-framed";
pub const HYBRID_SIGNATURE_SCHEME: &str = "ed25519+ml-dsa-65";
pub const ED25519_PUBLIC_KEY_LEN: usize = 32;
pub const ED25519_SIGNATURE_LEN: usize = 64;
pub const ML_DSA_65_PUBLIC_KEY_LEN: usize = 1952;
pub const ML_DSA_65_SIGNATURE_LEN: usize = 3309;
pub const MAX_POLICY_HEAD_LIFETIME_MS: u64 = 30_000;
pub const MAX_TRUST_KEYS_PER_ROLE: usize = 16;

const DOMAIN_TRUST_KEY: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-trust-key/v1";
const DOMAIN_ROOT: &[u8] = b"mycelix/authority/coordinator-release-key-policy-root/v1";
const DOMAIN_ROOT_BOOTSTRAP: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-root-bootstrap/v1";
const DOMAIN_ROOT_ROTATION: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-root-rotation/v1";
const DOMAIN_POLICY_HEAD: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-head/v1";
const DOMAIN_POLICY_HEAD_SIGNATURE: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-head-signature/v1";
const DOMAIN_SIGNER_SET: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-signer-set/v1";
const DOMAIN_ROOT_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-root-qualification/v1";
const DOMAIN_HEAD_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-head-qualification/v1";
const MAX_TEXT_BYTES: usize = 2048;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HybridPolicyTrustKey {
    pub key_id: String,
    pub ed25519_public_key: Vec<u8>,
    pub ml_dsa_65_public_key: Vec<u8>,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl HybridPolicyTrustKey {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
        validate_text(&self.key_id, "policy trust key id")?;
        if self.ed25519_public_key.len() != ED25519_PUBLIC_KEY_LEN
            || self.ed25519_public_key.iter().all(|byte| *byte == 0)
        {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidEd25519PublicKey);
        }
        if self.ml_dsa_65_public_key.len() != ML_DSA_65_PUBLIC_KEY_LEN
            || self.ml_dsa_65_public_key.iter().all(|byte| *byte == 0)
        {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidMlDsa65PublicKey);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidTrustKeyWindow);
        }
        Ok(())
    }

    pub fn key_digest(&self) -> Result<[u8; 32], CoordinatorReleaseKeyPolicyRootError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_TRUST_KEY);
        frame_hash(&mut hasher, TRUST_KEY_PROFILE.as_bytes());
        frame_hash(&mut hasher, HYBRID_SIGNATURE_SCHEME.as_bytes());
        frame_hash(&mut hasher, self.key_id.as_bytes());
        frame_hash(&mut hasher, &self.ed25519_public_key);
        frame_hash(&mut hasher, &self.ml_dsa_65_public_key);
        frame_hash(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame_hash(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }

    fn live_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HybridThresholdSignature {
    pub signature_profile: String,
    pub key_id: String,
    pub ed25519_signature: Vec<u8>,
    pub ml_dsa_65_signature: Vec<u8>,
}

impl HybridThresholdSignature {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
        if self.signature_profile != THRESHOLD_SIGNATURE_PROFILE {
            return Err(CoordinatorReleaseKeyPolicyRootError::WrongThresholdSignatureProfile);
        }
        validate_text(&self.key_id, "threshold signer key id")?;
        if self.ed25519_signature.len() != ED25519_SIGNATURE_LEN {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidEd25519SignatureLength);
        }
        if self.ml_dsa_65_signature.len() != ML_DSA_65_SIGNATURE_LEN {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidMlDsa65SignatureLength);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseKeyPolicyTrustRoot {
    pub protocol_version: String,
    pub root_id: String,
    pub root_version: u64,
    pub release_authority_ref: String,
    pub policy_id: String,
    pub root_keys: Vec<HybridPolicyTrustKey>,
    pub root_threshold: u16,
    pub policy_head_keys: Vec<HybridPolicyTrustKey>,
    pub policy_head_threshold: u16,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl CoordinatorReleaseKeyPolicyTrustRoot {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorReleaseKeyPolicyRootError::WrongProtocol);
        }
        validate_text(&self.root_id, "policy trust root id")?;
        validate_text(&self.release_authority_ref, "release authority ref")?;
        validate_text(&self.policy_id, "release key policy id")?;
        if self.root_version == 0 {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidRootVersion);
        }
        validate_role_keys(&self.root_keys, self.root_threshold)?;
        validate_role_keys(&self.policy_head_keys, self.policy_head_threshold)?;
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidRootWindow);
        }

        let mut ids = BTreeSet::new();
        let mut key_material = BTreeSet::new();
        for key in self.root_keys.iter().chain(self.policy_head_keys.iter()) {
            if !ids.insert(key.key_id.clone()) || !key_material.insert(key_material_digest(key)) {
                return Err(CoordinatorReleaseKeyPolicyRootError::CrossRoleKeyReuse);
            }
            if key.valid_until_ms <= self.valid_from_ms || key.valid_from_ms >= self.valid_until_ms {
                return Err(CoordinatorReleaseKeyPolicyRootError::TrustKeyOutsideRootWindow);
            }
        }
        Ok(())
    }

    pub fn root_digest(&self) -> Result<[u8; 32], CoordinatorReleaseKeyPolicyRootError> {
        self.validate()?;
        let mut roots: Vec<&HybridPolicyTrustKey> = self.root_keys.iter().collect();
        roots.sort_by(|a, b| a.key_id.cmp(&b.key_id));
        let mut policy: Vec<&HybridPolicyTrustKey> = self.policy_head_keys.iter().collect();
        policy.sort_by(|a, b| a.key_id.cmp(&b.key_id));

        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_ROOT);
        frame_hash(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame_hash(&mut hasher, ROOT_PROFILE.as_bytes());
        frame_hash(&mut hasher, HYBRID_SIGNATURE_SCHEME.as_bytes());
        frame_hash(&mut hasher, self.root_id.as_bytes());
        frame_hash(&mut hasher, &self.root_version.to_le_bytes());
        frame_hash(&mut hasher, self.release_authority_ref.as_bytes());
        frame_hash(&mut hasher, self.policy_id.as_bytes());
        frame_hash(&mut hasher, &self.root_threshold.to_le_bytes());
        for key in roots {
            frame_hash(&mut hasher, &key.key_digest()?);
            frame_hash(&mut hasher, TRUST_KEY_PROFILE.as_bytes());
        }
        frame_hash(&mut hasher, &self.policy_head_threshold.to_le_bytes());
        for key in policy {
            frame_hash(&mut hasher, &key.key_digest()?);
            frame_hash(&mut hasher, TRUST_KEY_PROFILE.as_bytes());
        }
        frame_hash(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame_hash(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }

    fn live_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseKeyPolicyRootPin {
    pub root_digest: [u8; 32],
    pub root_profile: String,
}

impl CoordinatorReleaseKeyPolicyRootPin {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
        validate_digest(&self.root_digest, "pinned release key-policy root digest")?;
        if self.root_profile != ROOT_PROFILE {
            return Err(CoordinatorReleaseKeyPolicyRootError::WrongRootProfile);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoordinatorReleaseKeyPolicyRoot {
    root: CoordinatorReleaseKeyPolicyTrustRoot,
    root_digest: [u8; 32],
    root_profile: String,
    qualification_digest: [u8; 32],
    qualification_profile: String,
    root_threshold_floor: u16,
    policy_head_threshold_floor: u16,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoordinatorReleaseKeyPolicyRoot {
    pub fn root(&self) -> &CoordinatorReleaseKeyPolicyTrustRoot {
        &self.root
    }
    pub fn root_digest(&self) -> [u8; 32] {
        self.root_digest
    }
    pub fn root_profile(&self) -> &str {
        &self.root_profile
    }
    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }
    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }
    pub fn verification_ref(&self) -> &str {
        &self.verification_ref
    }
    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }
    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    fn assert_live_at(&self, now_ms: u64) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
        if self.verified_at_ms > now_ms || self.valid_until_ms <= now_ms || !self.root.live_at(now_ms)
        {
            Err(CoordinatorReleaseKeyPolicyRootError::QualifiedRootNotLive)
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseKeyPolicyHead {
    pub protocol_version: String,
    pub root_version: u64,
    pub policy_digest: [u8; 32],
    pub policy_profile: String,
    pub policy_generation: u64,
    pub previous_head_digest: Option<[u8; 32]>,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl CoordinatorReleaseKeyPolicyHead {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorReleaseKeyPolicyRootError::WrongProtocol);
        }
        if self.root_version == 0 || self.policy_generation == 0 {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidPolicyHeadGeneration);
        }
        validate_digest(&self.policy_digest, "policy-head policy digest")?;
        if self.policy_profile != POLICY_PROFILE {
            return Err(CoordinatorReleaseKeyPolicyRootError::WrongPolicyProfile);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseKeyPolicyRootError::InvalidPolicyHeadWindow);
        }
        let width = self
            .valid_until_ms
            .checked_sub(self.valid_from_ms)
            .ok_or(CoordinatorReleaseKeyPolicyRootError::InvalidPolicyHeadWindow)?;
        if width > MAX_POLICY_HEAD_LIFETIME_MS {
            return Err(CoordinatorReleaseKeyPolicyRootError::PolicyHeadLifetimeTooWide);
        }
        if let Some(previous) = self.previous_head_digest {
            validate_digest(&previous, "policy-head predecessor digest")?;
        }
        Ok(())
    }

    pub fn head_digest(&self) -> Result<[u8; 32], CoordinatorReleaseKeyPolicyRootError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_POLICY_HEAD);
        frame_hash(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame_hash(&mut hasher, POLICY_HEAD_PROFILE.as_bytes());
        frame_hash(&mut hasher, &self.root_version.to_le_bytes());
        frame_hash(&mut hasher, &self.policy_digest);
        frame_hash(&mut hasher, self.policy_profile.as_bytes());
        frame_hash(&mut hasher, &self.policy_generation.to_le_bytes());
        match self.previous_head_digest {
            Some(previous) => {
                frame_hash(&mut hasher, b"previous");
                frame_hash(&mut hasher, &previous);
            }
            None => frame_hash(&mut hasher, b"genesis"),
        }
        frame_hash(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame_hash(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCurrentCoordinatorReleaseKeyPolicyHead {
    policy: CoordinatorReleaseKeyPolicy,
    head: CoordinatorReleaseKeyPolicyHead,
    root_digest: [u8; 32],
    root_profile: String,
    head_digest: [u8; 32],
    head_profile: String,
    signer_set_digest: [u8; 32],
    qualification_digest: [u8; 32],
    qualification_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentCoordinatorReleaseKeyPolicyHead {
    pub fn policy(&self) -> &CoordinatorReleaseKeyPolicy {
        &self.policy
    }
    pub fn head(&self) -> &CoordinatorReleaseKeyPolicyHead {
        &self.head
    }
    pub fn root_digest(&self) -> [u8; 32] {
        self.root_digest
    }
    pub fn root_profile(&self) -> &str {
        &self.root_profile
    }
    pub fn head_digest(&self) -> [u8; 32] {
        self.head_digest
    }
    pub fn head_profile(&self) -> &str {
        &self.head_profile
    }
    pub fn signer_set_digest(&self) -> [u8; 32] {
        self.signer_set_digest
    }
    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }
    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }
    pub fn verification_ref(&self) -> &str {
        &self.verification_ref
    }
    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }
    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    /// Build #307 currentness privately and consume it immediately. The public live
    /// path never accepts caller-supplied `VerifiedCurrent...` bytes.
    pub fn qualify_manifest_key_policy(
        &self,
        manifest: &CoordinatorReleaseManifest,
    ) -> Result<QualifiedCoordinatorReleaseManifestKeyPolicy, CoordinatorReleaseKeyPolicyRootError>
    {
        let now_ms = system_time_ms()?;
        if self.verified_at_ms > now_ms || self.valid_until_ms <= now_ms {
            return Err(CoordinatorReleaseKeyPolicyRootError::QualifiedPolicyHeadNotLive);
        }
        let receipt = VerifiedCurrentCoordinatorReleaseKeyPolicyProof {
            protocol_version: CURRENT_POLICY_PROOF_PROTOCOL.into(),
            policy_digest: self.head.policy_digest,
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: self.head.policy_generation,
            source_ref: format!(
                "coordinator-release-key-policy-head-blake3:{}",
                encode_hex(&self.head_digest)
            ),
            verifier_ref: self.verification_ref.clone(),
            verified_at_ms: self.verified_at_ms,
            valid_until_ms: self.valid_until_ms,
        };
        qualify_manifest_key_policy_v01(manifest, self.policy.clone(), &receipt, now_ms)
            .map_err(|error| CoordinatorReleaseKeyPolicyRootError::ManifestPolicy(error.to_string()))
    }
}

pub fn root_bootstrap_signature_message(
    root: &CoordinatorReleaseKeyPolicyTrustRoot,
) -> Result<Vec<u8>, CoordinatorReleaseKeyPolicyRootError> {
    let mut out = Vec::new();
    frame_vec(&mut out, DOMAIN_ROOT_BOOTSTRAP);
    frame_vec(&mut out, PROTOCOL_VERSION.as_bytes());
    frame_vec(&mut out, ROOT_PROFILE.as_bytes());
    frame_vec(&mut out, &root.root_digest()?);
    Ok(out)
}

pub fn root_rotation_signature_message(
    old_root: &CoordinatorReleaseKeyPolicyTrustRoot,
    new_root: &CoordinatorReleaseKeyPolicyTrustRoot,
) -> Result<Vec<u8>, CoordinatorReleaseKeyPolicyRootError> {
    old_root.validate()?;
    new_root.validate()?;
    let mut out = Vec::new();
    frame_vec(&mut out, DOMAIN_ROOT_ROTATION);
    frame_vec(&mut out, ROOT_ROTATION_PROFILE.as_bytes());
    frame_vec(&mut out, &old_root.root_digest()?);
    frame_vec(&mut out, &new_root.root_digest()?);
    frame_vec(&mut out, &old_root.root_version.to_le_bytes());
    frame_vec(&mut out, &new_root.root_version.to_le_bytes());
    Ok(out)
}

pub fn policy_head_signature_message(
    root: &CoordinatorReleaseKeyPolicyTrustRoot,
    head: &CoordinatorReleaseKeyPolicyHead,
) -> Result<Vec<u8>, CoordinatorReleaseKeyPolicyRootError> {
    root.validate()?;
    head.validate()?;
    let mut out = Vec::new();
    frame_vec(&mut out, DOMAIN_POLICY_HEAD_SIGNATURE);
    frame_vec(&mut out, POLICY_HEAD_PROFILE.as_bytes());
    frame_vec(&mut out, &root.root_digest()?);
    frame_vec(&mut out, &head.head_digest()?);
    Ok(out)
}

/// Bootstrap from a root fingerprint established through an independent out-of-band
/// channel. Pin equality establishes identity only; it never widens root lifetime.
pub fn bootstrap_policy_root(
    root: CoordinatorReleaseKeyPolicyTrustRoot,
    pin: &CoordinatorReleaseKeyPolicyRootPin,
    signatures: &[HybridThresholdSignature],
) -> Result<QualifiedCoordinatorReleaseKeyPolicyRoot, CoordinatorReleaseKeyPolicyRootError> {
    root.validate()?;
    pin.validate()?;
    let root_digest = root.root_digest()?;
    if pin.root_digest != root_digest || pin.root_profile != ROOT_PROFILE {
        return Err(CoordinatorReleaseKeyPolicyRootError::RootPinMismatch);
    }

    let started_at_ms = system_time_ms()?;
    if !root.live_at(started_at_ms) {
        return Err(CoordinatorReleaseKeyPolicyRootError::RootNotLive);
    }
    let message = root_bootstrap_signature_message(&root)?;
    let signers = verify_exact_threshold(
        &root.root_keys,
        root.root_threshold,
        signatures,
        &message,
        started_at_ms,
    )?;
    let verified_at_ms = system_time_ms()?;
    ensure_signers_live(&root.root_keys, signatures, verified_at_ms)?;
    if !root.live_at(verified_at_ms) {
        return Err(CoordinatorReleaseKeyPolicyRootError::RootNotLiveAfterVerification);
    }

    let signer_set_digest = signer_set_digest(&signers);
    let valid_until_ms = root.valid_until_ms;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_ROOT_QUALIFICATION);
    frame_hash(&mut hasher, ROOT_QUALIFICATION_PROFILE.as_bytes());
    frame_hash(&mut hasher, &root_digest);
    frame_hash(&mut hasher, ROOT_PROFILE.as_bytes());
    frame_hash(&mut hasher, &pin.root_digest);
    frame_hash(&mut hasher, pin.root_profile.as_bytes());
    frame_hash(&mut hasher, &signer_set_digest);
    frame_hash(&mut hasher, &verified_at_ms.to_le_bytes());
    frame_hash(&mut hasher, &valid_until_ms.to_le_bytes());
    let qualification_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCoordinatorReleaseKeyPolicyRoot {
        root_threshold_floor: root.root_threshold,
        policy_head_threshold_floor: root.policy_head_threshold,
        verification_ref: format!(
            "coordinator-release-key-policy-root-blake3:{}",
            encode_hex(&qualification_digest)
        ),
        root,
        root_digest,
        root_profile: ROOT_PROFILE.into(),
        qualification_digest,
        qualification_profile: ROOT_QUALIFICATION_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

/// Rotate a root only when the same transition is authorized by both the old and new
/// root thresholds. In-band rotation cannot weaken the bootstrap threshold floors.
pub fn rotate_policy_root(
    current: &QualifiedCoordinatorReleaseKeyPolicyRoot,
    new_root: CoordinatorReleaseKeyPolicyTrustRoot,
    old_root_signatures: &[HybridThresholdSignature],
    new_root_signatures: &[HybridThresholdSignature],
) -> Result<QualifiedCoordinatorReleaseKeyPolicyRoot, CoordinatorReleaseKeyPolicyRootError> {
    let started_at_ms = system_time_ms()?;
    current.assert_live_at(started_at_ms)?;
    new_root.validate()?;
    if new_root.root_id != current.root.root_id
        || new_root.release_authority_ref != current.root.release_authority_ref
        || new_root.policy_id != current.root.policy_id
    {
        return Err(CoordinatorReleaseKeyPolicyRootError::RootScopeChanged);
    }
    let expected_version = current
        .root
        .root_version
        .checked_add(1)
        .ok_or(CoordinatorReleaseKeyPolicyRootError::InvalidRootRotationVersion)?;
    if new_root.root_version != expected_version {
        return Err(CoordinatorReleaseKeyPolicyRootError::InvalidRootRotationVersion);
    }
    if new_root.root_threshold < current.root_threshold_floor
        || new_root.policy_head_threshold < current.policy_head_threshold_floor
    {
        return Err(CoordinatorReleaseKeyPolicyRootError::ThresholdFloorWeakened);
    }
    if !new_root.live_at(started_at_ms) {
        return Err(CoordinatorReleaseKeyPolicyRootError::RootNotLive);
    }

    let message = root_rotation_signature_message(&current.root, &new_root)?;
    let old_signers = verify_exact_threshold(
        &current.root.root_keys,
        current.root.root_threshold,
        old_root_signatures,
        &message,
        started_at_ms,
    )?;
    let new_signers = verify_exact_threshold(
        &new_root.root_keys,
        new_root.root_threshold,
        new_root_signatures,
        &message,
        started_at_ms,
    )?;
    let verified_at_ms = system_time_ms()?;
    current.assert_live_at(verified_at_ms)?;
    ensure_signers_live(&current.root.root_keys, old_root_signatures, verified_at_ms)?;
    ensure_signers_live(&new_root.root_keys, new_root_signatures, verified_at_ms)?;
    if !new_root.live_at(verified_at_ms) {
        return Err(CoordinatorReleaseKeyPolicyRootError::RootNotLiveAfterVerification);
    }

    let new_root_digest = new_root.root_digest()?;
    let valid_until_ms = new_root.valid_until_ms;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_ROOT_QUALIFICATION);
    frame_hash(&mut hasher, ROOT_QUALIFICATION_PROFILE.as_bytes());
    frame_hash(&mut hasher, &current.root_digest);
    frame_hash(&mut hasher, &new_root_digest);
    frame_hash(&mut hasher, ROOT_ROTATION_PROFILE.as_bytes());
    frame_hash(&mut hasher, &signer_set_digest(&old_signers));
    frame_hash(&mut hasher, &signer_set_digest(&new_signers));
    frame_hash(&mut hasher, &verified_at_ms.to_le_bytes());
    frame_hash(&mut hasher, &valid_until_ms.to_le_bytes());
    let qualification_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCoordinatorReleaseKeyPolicyRoot {
        root_threshold_floor: current.root_threshold_floor,
        policy_head_threshold_floor: current.policy_head_threshold_floor,
        verification_ref: format!(
            "coordinator-release-key-policy-root-blake3:{}",
            encode_hex(&qualification_digest)
        ),
        root: new_root,
        root_digest: new_root_digest,
        root_profile: ROOT_PROFILE.into(),
        qualification_digest,
        qualification_profile: ROOT_QUALIFICATION_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

/// Qualify the exact short-lived policy head. A predecessor-free head is legal only
/// for root version 1 / policy generation 1; rotated roots must continue the exact
/// already-qualified predecessor chain.
pub fn qualify_current_policy_head(
    root: &QualifiedCoordinatorReleaseKeyPolicyRoot,
    policy: CoordinatorReleaseKeyPolicy,
    head: CoordinatorReleaseKeyPolicyHead,
    signatures: &[HybridThresholdSignature],
    previous: Option<&QualifiedCurrentCoordinatorReleaseKeyPolicyHead>,
) -> Result<QualifiedCurrentCoordinatorReleaseKeyPolicyHead, CoordinatorReleaseKeyPolicyRootError>
{
    let started_at_ms = system_time_ms()?;
    root.assert_live_at(started_at_ms)?;
    policy
        .validate()
        .map_err(|error| CoordinatorReleaseKeyPolicyRootError::SemanticPolicy(error.to_string()))?;
    head.validate()?;

    if policy.release_authority_ref != root.root.release_authority_ref
        || policy.policy_id != root.root.policy_id
    {
        return Err(CoordinatorReleaseKeyPolicyRootError::PolicyOutsideRootScope);
    }
    let policy_digest = policy
        .policy_digest()
        .map_err(|error| CoordinatorReleaseKeyPolicyRootError::SemanticPolicy(error.to_string()))?;
    if head.root_version != root.root.root_version
        || head.policy_digest != policy_digest
        || head.policy_profile != POLICY_PROFILE
        || head.policy_generation != policy.policy_generation
    {
        return Err(CoordinatorReleaseKeyPolicyRootError::PolicyHeadBindingMismatch);
    }
    if started_at_ms < policy.valid_from_ms || started_at_ms >= policy.valid_until_ms {
        return Err(CoordinatorReleaseKeyPolicyRootError::SemanticPolicyNotLive);
    }
    if started_at_ms < head.valid_from_ms || started_at_ms >= head.valid_until_ms {
        return Err(CoordinatorReleaseKeyPolicyRootError::PolicyHeadNotLive);
    }

    match previous {
        None => {
            if root.root.root_version != 1
                || head.policy_generation != 1
                || head.previous_head_digest.is_some()
            {
                return Err(CoordinatorReleaseKeyPolicyRootError::InvalidPolicyHeadGenesis);
            }
        }
        Some(previous) => {
            if previous.policy.policy_id != policy.policy_id
                || previous.policy.release_authority_ref != policy.release_authority_ref
            {
                return Err(CoordinatorReleaseKeyPolicyRootError::PolicyHeadPredecessorMismatch);
            }
            let expected_generation = previous
                .head
                .policy_generation
                .checked_add(1)
                .ok_or(CoordinatorReleaseKeyPolicyRootError::PolicyHeadGenerationOverflow)?;
            if head.policy_generation != expected_generation
                || head.previous_head_digest != Some(previous.head_digest)
            {
                return Err(CoordinatorReleaseKeyPolicyRootError::PolicyHeadPredecessorMismatch);
            }
        }
    }

    let message = policy_head_signature_message(&root.root, &head)?;
    let signers = verify_exact_threshold(
        &root.root.policy_head_keys,
        root.root.policy_head_threshold,
        signatures,
        &message,
        started_at_ms,
    )?;
    let verified_at_ms = system_time_ms()?;
    root.assert_live_at(verified_at_ms)?;
    ensure_signers_live(&root.root.policy_head_keys, signatures, verified_at_ms)?;
    if verified_at_ms < policy.valid_from_ms || verified_at_ms >= policy.valid_until_ms {
        return Err(CoordinatorReleaseKeyPolicyRootError::SemanticPolicyNotLiveAfterVerification);
    }
    if verified_at_ms < head.valid_from_ms || verified_at_ms >= head.valid_until_ms {
        return Err(CoordinatorReleaseKeyPolicyRootError::PolicyHeadNotLiveAfterVerification);
    }

    let valid_until_ms = root
        .valid_until_ms
        .min(policy.valid_until_ms)
        .min(head.valid_until_ms)
        .min(signer_horizon(&root.root.policy_head_keys, signatures)?);
    if valid_until_ms <= verified_at_ms {
        return Err(CoordinatorReleaseKeyPolicyRootError::EmptyPolicyHeadWindow);
    }

    let head_digest = head.head_digest()?;
    let signer_set_digest = signer_set_digest(&signers);
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_HEAD_QUALIFICATION);
    frame_hash(&mut hasher, POLICY_HEAD_QUALIFICATION_PROFILE.as_bytes());
    frame_hash(&mut hasher, &root.root_digest);
    frame_hash(&mut hasher, ROOT_PROFILE.as_bytes());
    frame_hash(&mut hasher, &head_digest);
    frame_hash(&mut hasher, POLICY_HEAD_PROFILE.as_bytes());
    frame_hash(&mut hasher, &policy_digest);
    frame_hash(&mut hasher, POLICY_PROFILE.as_bytes());
    frame_hash(&mut hasher, &signer_set_digest);
    frame_hash(&mut hasher, &verified_at_ms.to_le_bytes());
    frame_hash(&mut hasher, &valid_until_ms.to_le_bytes());
    let qualification_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCurrentCoordinatorReleaseKeyPolicyHead {
        policy,
        head,
        root_digest: root.root_digest,
        root_profile: ROOT_PROFILE.into(),
        head_digest,
        head_profile: POLICY_HEAD_PROFILE.into(),
        signer_set_digest,
        qualification_digest,
        qualification_profile: POLICY_HEAD_QUALIFICATION_PROFILE.into(),
        verification_ref: format!(
            "coordinator-release-key-policy-rooted-head-blake3:{}",
            encode_hex(&qualification_digest)
        ),
        verified_at_ms,
        valid_until_ms,
    })
}

fn validate_role_keys(
    keys: &[HybridPolicyTrustKey],
    threshold: u16,
) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
    if keys.is_empty() || keys.len() > MAX_TRUST_KEYS_PER_ROLE {
        return Err(CoordinatorReleaseKeyPolicyRootError::InvalidTrustKeySetSize);
    }
    if threshold == 0 || usize::from(threshold) > keys.len() {
        return Err(CoordinatorReleaseKeyPolicyRootError::InvalidThreshold);
    }
    let mut ids = BTreeSet::new();
    let mut material = BTreeSet::new();
    for key in keys {
        key.validate()?;
        if !ids.insert(key.key_id.clone()) || !material.insert(key_material_digest(key)) {
            return Err(CoordinatorReleaseKeyPolicyRootError::DuplicateTrustKey);
        }
    }
    Ok(())
}

fn verify_exact_threshold(
    keys: &[HybridPolicyTrustKey],
    threshold: u16,
    signatures: &[HybridThresholdSignature],
    message: &[u8],
    now_ms: u64,
) -> Result<Vec<String>, CoordinatorReleaseKeyPolicyRootError> {
    if signatures.len() != usize::from(threshold) {
        return Err(CoordinatorReleaseKeyPolicyRootError::ThresholdSignatureCountMismatch);
    }
    let mut signer_ids = BTreeSet::new();
    for signature in signatures {
        signature.validate()?;
        if !signer_ids.insert(signature.key_id.clone()) {
            return Err(CoordinatorReleaseKeyPolicyRootError::DuplicateThresholdSigner);
        }
        let key = keys
            .iter()
            .find(|candidate| candidate.key_id == signature.key_id)
            .ok_or(CoordinatorReleaseKeyPolicyRootError::UnauthorizedThresholdSigner)?;
        if !key.live_at(now_ms) {
            return Err(CoordinatorReleaseKeyPolicyRootError::ThresholdSignerNotLive);
        }
        verify_hybrid(key, message, signature)?;
    }
    Ok(signer_ids.into_iter().collect())
}

fn ensure_signers_live(
    keys: &[HybridPolicyTrustKey],
    signatures: &[HybridThresholdSignature],
    now_ms: u64,
) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
    for signature in signatures {
        let key = keys
            .iter()
            .find(|candidate| candidate.key_id == signature.key_id)
            .ok_or(CoordinatorReleaseKeyPolicyRootError::UnauthorizedThresholdSigner)?;
        if !key.live_at(now_ms) {
            return Err(CoordinatorReleaseKeyPolicyRootError::ThresholdSignerExpiredDuringVerification);
        }
    }
    Ok(())
}

fn signer_horizon(
    keys: &[HybridPolicyTrustKey],
    signatures: &[HybridThresholdSignature],
) -> Result<u64, CoordinatorReleaseKeyPolicyRootError> {
    signatures
        .iter()
        .map(|signature| {
            keys.iter()
                .find(|candidate| candidate.key_id == signature.key_id)
                .map(|key| key.valid_until_ms)
                .ok_or(CoordinatorReleaseKeyPolicyRootError::UnauthorizedThresholdSigner)
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .min()
        .ok_or(CoordinatorReleaseKeyPolicyRootError::ThresholdSignatureCountMismatch)
}

fn verify_hybrid(
    key: &HybridPolicyTrustKey,
    message: &[u8],
    signature: &HybridThresholdSignature,
) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
    let ed_key: [u8; ED25519_PUBLIC_KEY_LEN] = key
        .ed25519_public_key
        .as_slice()
        .try_into()
        .map_err(|_| CoordinatorReleaseKeyPolicyRootError::InvalidEd25519PublicKey)?;
    let ed_signature: [u8; ED25519_SIGNATURE_LEN] = signature
        .ed25519_signature
        .as_slice()
        .try_into()
        .map_err(|_| CoordinatorReleaseKeyPolicyRootError::InvalidEd25519SignatureLength)?;
    let verifying_key = EdVerifyingKey::from_bytes(&ed_key)
        .map_err(|_| CoordinatorReleaseKeyPolicyRootError::InvalidEd25519PublicKey)?;
    verifying_key
        .verify_strict(message, &EdSignature::from_bytes(&ed_signature))
        .map_err(|_| CoordinatorReleaseKeyPolicyRootError::Ed25519VerificationFailed)?;

    let encoded_key = EncodedVerifyingKey::<MlDsa65>::try_from(key.ml_dsa_65_public_key.as_slice())
        .map_err(|_| CoordinatorReleaseKeyPolicyRootError::InvalidMlDsa65PublicKey)?;
    let ml_key = MlVerifyingKey::<MlDsa65>::decode(&encoded_key);
    let encoded_signature =
        EncodedSignature::<MlDsa65>::try_from(signature.ml_dsa_65_signature.as_slice())
            .map_err(|_| CoordinatorReleaseKeyPolicyRootError::InvalidMlDsa65SignatureLength)?;
    let ml_signature = MlSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or(CoordinatorReleaseKeyPolicyRootError::InvalidMlDsa65Signature)?;
    ml_key
        .verify(message, &ml_signature)
        .map_err(|_| CoordinatorReleaseKeyPolicyRootError::MlDsa65VerificationFailed)
}

fn signer_set_digest(ids: &[String]) -> [u8; 32] {
    let mut ids = ids.to_vec();
    ids.sort();
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_SIGNER_SET);
    for id in ids {
        frame_hash(&mut hasher, id.as_bytes());
    }
    *hasher.finalize().as_bytes()
}

fn key_material_digest(key: &HybridPolicyTrustKey) -> [u8; 32] {
    let mut hasher = blake3::Hasher::new();
    hasher.update(b"mycelix/authority/coordinator-release-key-policy-trust-key-material/v1");
    frame_hash(&mut hasher, &key.ed25519_public_key);
    frame_hash(&mut hasher, &key.ml_dsa_65_public_key);
    *hasher.finalize().as_bytes()
}

fn system_time_ms() -> Result<u64, CoordinatorReleaseKeyPolicyRootError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| CoordinatorReleaseKeyPolicyRootError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| CoordinatorReleaseKeyPolicyRootError::ClockOverflow)
}

fn validate_text(
    value: &str,
    field: &'static str,
) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(CoordinatorReleaseKeyPolicyRootError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn validate_digest(
    value: &[u8; 32],
    field: &'static str,
) -> Result<(), CoordinatorReleaseKeyPolicyRootError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorReleaseKeyPolicyRootError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn frame_vec(target: &mut Vec<u8>, bytes: &[u8]) {
    target.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    target.extend_from_slice(bytes);
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorReleaseKeyPolicyRootError {
    WrongProtocol,
    WrongRootProfile,
    WrongPolicyProfile,
    WrongThresholdSignatureProfile,
    InvalidText(&'static str),
    InvalidDigest(&'static str),
    InvalidRootVersion,
    InvalidTrustKeySetSize,
    InvalidThreshold,
    DuplicateTrustKey,
    CrossRoleKeyReuse,
    TrustKeyOutsideRootWindow,
    InvalidEd25519PublicKey,
    InvalidMlDsa65PublicKey,
    InvalidTrustKeyWindow,
    InvalidEd25519SignatureLength,
    InvalidMlDsa65SignatureLength,
    InvalidMlDsa65Signature,
    Ed25519VerificationFailed,
    MlDsa65VerificationFailed,
    InvalidRootWindow,
    RootPinMismatch,
    RootNotLive,
    RootNotLiveAfterVerification,
    QualifiedRootNotLive,
    ThresholdSignatureCountMismatch,
    DuplicateThresholdSigner,
    UnauthorizedThresholdSigner,
    ThresholdSignerNotLive,
    ThresholdSignerExpiredDuringVerification,
    RootScopeChanged,
    InvalidRootRotationVersion,
    ThresholdFloorWeakened,
    InvalidPolicyHeadGeneration,
    InvalidPolicyHeadWindow,
    PolicyHeadLifetimeTooWide,
    InvalidPolicyHeadGenesis,
    PolicyHeadGenerationOverflow,
    PolicyHeadPredecessorMismatch,
    PolicyOutsideRootScope,
    PolicyHeadBindingMismatch,
    SemanticPolicy(String),
    SemanticPolicyNotLive,
    SemanticPolicyNotLiveAfterVerification,
    PolicyHeadNotLive,
    PolicyHeadNotLiveAfterVerification,
    EmptyPolicyHeadWindow,
    QualifiedPolicyHeadNotLive,
    ManifestPolicy(String),
    ClockBeforeUnixEpoch,
    ClockOverflow,
}

impl fmt::Display for CoordinatorReleaseKeyPolicyRootError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use CoordinatorReleaseKeyPolicyRootError as E;
        match self {
            E::WrongProtocol => write!(f, "wrong coordinator release key-policy root protocol"),
            E::WrongRootProfile => write!(f, "wrong coordinator release key-policy root profile"),
            E::WrongPolicyProfile => write!(f, "wrong coordinator release key-policy profile"),
            E::WrongThresholdSignatureProfile => write!(f, "wrong hybrid threshold-signature profile"),
            E::InvalidText(field) => write!(f, "invalid {field}"),
            E::InvalidDigest(field) => write!(f, "invalid {field}"),
            E::InvalidRootVersion => write!(f, "policy trust-root version must be non-zero"),
            E::InvalidTrustKeySetSize => write!(f, "policy trust role contains an invalid number of keys"),
            E::InvalidThreshold => write!(f, "policy trust threshold is invalid"),
            E::DuplicateTrustKey => write!(f, "policy trust role contains duplicate key identity/material"),
            E::CrossRoleKeyReuse => write!(f, "root and policy-head roles must use disjoint key identity/material"),
            E::TrustKeyOutsideRootWindow => write!(f, "policy trust key does not overlap root lifetime"),
            E::InvalidEd25519PublicKey => write!(f, "invalid 32-byte Ed25519 public key"),
            E::InvalidMlDsa65PublicKey => write!(f, "invalid 1952-byte ML-DSA-65 public key"),
            E::InvalidTrustKeyWindow => write!(f, "policy trust-key lifetime is invalid"),
            E::InvalidEd25519SignatureLength => write!(f, "Ed25519 threshold signature must be 64 bytes"),
            E::InvalidMlDsa65SignatureLength => write!(f, "ML-DSA-65 threshold signature must be 3309 bytes"),
            E::InvalidMlDsa65Signature => write!(f, "ML-DSA-65 threshold signature is undecodable"),
            E::Ed25519VerificationFailed => write!(f, "Ed25519 threshold verification failed"),
            E::MlDsa65VerificationFailed => write!(f, "ML-DSA-65 threshold verification failed"),
            E::InvalidRootWindow => write!(f, "policy trust-root lifetime is invalid"),
            E::RootPinMismatch => write!(f, "out-of-band root pin does not match candidate root"),
            E::RootNotLive => write!(f, "candidate policy trust root is not live"),
            E::RootNotLiveAfterVerification => write!(f, "candidate root expired during verification"),
            E::QualifiedRootNotLive => write!(f, "qualified policy trust root is not live"),
            E::ThresholdSignatureCountMismatch => write!(f, "threshold evidence must contain exactly the configured signature count"),
            E::DuplicateThresholdSigner => write!(f, "threshold evidence repeats a signer"),
            E::UnauthorizedThresholdSigner => write!(f, "threshold evidence contains an unauthorized signer"),
            E::ThresholdSignerNotLive => write!(f, "threshold signer is not live at verification start"),
            E::ThresholdSignerExpiredDuringVerification => write!(f, "threshold signer expired during verification"),
            E::RootScopeChanged => write!(f, "root rotation cannot change root id, release authority or policy id"),
            E::InvalidRootRotationVersion => write!(f, "root rotation must advance version by exactly one"),
            E::ThresholdFloorWeakened => write!(f, "root rotation cannot lower bootstrap threshold floors"),
            E::InvalidPolicyHeadGeneration => write!(f, "policy-head root version/generation is invalid"),
            E::InvalidPolicyHeadWindow => write!(f, "policy-head lifetime is invalid"),
            E::PolicyHeadLifetimeTooWide => write!(f, "policy-head lifetime exceeds the v0.1 cap"),
            E::InvalidPolicyHeadGenesis => write!(f, "only root-v1/policy-generation-1 may omit a predecessor"),
            E::PolicyHeadGenerationOverflow => write!(f, "policy-head generation overflow"),
            E::PolicyHeadPredecessorMismatch => write!(f, "policy head does not extend the exact trusted predecessor"),
            E::PolicyOutsideRootScope => write!(f, "semantic policy is outside rooted authority/policy scope"),
            E::PolicyHeadBindingMismatch => write!(f, "policy head does not bind the exact root/policy generation"),
            E::SemanticPolicy(error) => write!(f, "invalid semantic release key policy: {error}"),
            E::SemanticPolicyNotLive => write!(f, "semantic release key policy is not live"),
            E::SemanticPolicyNotLiveAfterVerification => write!(f, "semantic policy expired during verification"),
            E::PolicyHeadNotLive => write!(f, "release key-policy head is not live"),
            E::PolicyHeadNotLiveAfterVerification => write!(f, "policy head expired during verification"),
            E::EmptyPolicyHeadWindow => write!(f, "qualified policy head has no live window"),
            E::QualifiedPolicyHeadNotLive => write!(f, "qualified policy head is not live"),
            E::ManifestPolicy(error) => write!(f, "#307 manifest/key-policy qualification failed: {error}"),
            E::ClockBeforeUnixEpoch => write!(f, "system clock is before Unix epoch"),
            E::ClockOverflow => write!(f, "policy trust verifier clock overflow"),
        }
    }
}

impl std::error::Error for CoordinatorReleaseKeyPolicyRootError {}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as EdSigner, SigningKey as EdSigningKey};
    use ml_dsa::signature::{Keypair as _, Signer as MlSigner};
    use ml_dsa::{Generate as _, KeyExport as _, SigningKey as MlSigningKey};
    use mycelix_authority_coordinator_deployment::CoordinatorCodeIdentity;
    use mycelix_authority_coordinator_release::{
        CoordinatorReleaseManifest, PROTOCOL_VERSION as RELEASE_PROTOCOL_VERSION,
    };
    use mycelix_authority_coordinator_release_key_policy::{
        AuthorizedHybridReleaseKey, CoordinatorReleaseKeyPolicy,
        PROTOCOL_VERSION as KEY_POLICY_PROTOCOL,
    };

    struct TestSigner {
        id: String,
        ed: EdSigningKey,
        ml: MlSigningKey<MlDsa65>,
    }

    impl TestSigner {
        fn new(id: &str, seed: u8) -> Self {
            Self {
                id: id.into(),
                ed: EdSigningKey::from_bytes(&[seed; 32]),
                ml: MlSigningKey::<MlDsa65>::generate(),
            }
        }

        fn trust_key(&self, start: u64, end: u64) -> HybridPolicyTrustKey {
            HybridPolicyTrustKey {
                key_id: self.id.clone(),
                ed25519_public_key: self.ed.verifying_key().to_bytes().to_vec(),
                ml_dsa_65_public_key: self.ml.verifying_key().encode().as_slice().to_vec(),
                valid_from_ms: start,
                valid_until_ms: end,
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

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }
    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn release_key(now: u64) -> AuthorizedHybridReleaseKey {
        AuthorizedHybridReleaseKey {
            key_id: "release-key-1".into(),
            ed25519_public_key: vec![7; ED25519_PUBLIC_KEY_LEN],
            ml_dsa_65_public_key: vec![8; ML_DSA_65_PUBLIC_KEY_LEN],
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 120_000,
        }
    }

    fn policy(now: u64, generation: u64) -> CoordinatorReleaseKeyPolicy {
        CoordinatorReleaseKeyPolicy {
            protocol_version: KEY_POLICY_PROTOCOL.into(),
            policy_id: "coordinator-release-keys".into(),
            policy_generation: generation,
            release_authority_ref: "release-authority:offline".into(),
            authorized_keys: vec![release_key(now)],
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 120_000,
        }
    }

    fn manifest(policy: &CoordinatorReleaseKeyPolicy, now: u64) -> CoordinatorReleaseManifest {
        CoordinatorReleaseManifest {
            protocol_version: RELEASE_PROTOCOL_VERSION.into(),
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
            valid_until_ms: now + 120_000,
        }
    }

    fn root(
        now: u64,
        version: u64,
        roots: &[TestSigner],
        policies: &[TestSigner],
        root_threshold: u16,
        policy_threshold: u16,
    ) -> CoordinatorReleaseKeyPolicyTrustRoot {
        CoordinatorReleaseKeyPolicyTrustRoot {
            protocol_version: PROTOCOL_VERSION.into(),
            root_id: "coordinator-release-policy-root".into(),
            root_version: version,
            release_authority_ref: "release-authority:offline".into(),
            policy_id: "coordinator-release-keys".into(),
            root_keys: roots
                .iter()
                .map(|s| s.trust_key(now.saturating_sub(5_000), now + 120_000))
                .collect(),
            root_threshold,
            policy_head_keys: policies
                .iter()
                .map(|s| s.trust_key(now.saturating_sub(5_000), now + 120_000))
                .collect(),
            policy_head_threshold: policy_threshold,
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 120_000,
        }
    }

    fn bootstrap(
        candidate: CoordinatorReleaseKeyPolicyTrustRoot,
        root_signers: &[TestSigner],
    ) -> QualifiedCoordinatorReleaseKeyPolicyRoot {
        let pin = CoordinatorReleaseKeyPolicyRootPin {
            root_digest: candidate.root_digest().unwrap(),
            root_profile: ROOT_PROFILE.into(),
        };
        let message = root_bootstrap_signature_message(&candidate).unwrap();
        let signatures: Vec<_> = root_signers
            .iter()
            .take(usize::from(candidate.root_threshold))
            .map(|signer| signer.sign(&message))
            .collect();
        bootstrap_policy_root(candidate, &pin, &signatures).unwrap()
    }

    #[test]
    fn rooted_short_lived_head_authorizes_manifest_policy_without_release_signature() {
        let now = system_time_ms().unwrap();
        let roots = vec![TestSigner::new("r1", 1), TestSigner::new("r2", 2)];
        let policy_signers = vec![TestSigner::new("p1", 11), TestSigner::new("p2", 12)];
        let qualified_root = bootstrap(root(now, 1, &roots, &policy_signers, 2, 2), &roots);
        let semantic = policy(now, 1);
        let head = CoordinatorReleaseKeyPolicyHead {
            protocol_version: PROTOCOL_VERSION.into(),
            root_version: 1,
            policy_digest: semantic.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: 1,
            previous_head_digest: None,
            valid_from_ms: now.saturating_sub(1_000),
            valid_until_ms: now + 20_000,
        };
        let message = policy_head_signature_message(qualified_root.root(), &head).unwrap();
        let signatures = vec![policy_signers[0].sign(&message), policy_signers[1].sign(&message)];
        let current = qualify_current_policy_head(
            &qualified_root,
            semantic.clone(),
            head,
            &signatures,
            None,
        )
        .unwrap();
        let candidate = manifest(&semantic, now);
        let qualified = current.qualify_manifest_key_policy(&candidate).unwrap();
        let key = qualified
            .qualify_key("release-key-1", system_time_ms().unwrap())
            .unwrap();
        assert_eq!(key.manifest_digest(), candidate.manifest_digest().unwrap());
    }

    #[test]
    fn wrong_pin_and_insufficient_root_threshold_deny() {
        let now = system_time_ms().unwrap();
        let roots = vec![TestSigner::new("r1", 21), TestSigner::new("r2", 22)];
        let policies = vec![TestSigner::new("p1", 31), TestSigner::new("p2", 32)];
        let candidate = root(now, 1, &roots, &policies, 2, 2);
        let message = root_bootstrap_signature_message(&candidate).unwrap();
        let signatures = vec![roots[0].sign(&message), roots[1].sign(&message)];
        let wrong_pin = CoordinatorReleaseKeyPolicyRootPin {
            root_digest: d(99),
            root_profile: ROOT_PROFILE.into(),
        };
        assert_eq!(
            bootstrap_policy_root(candidate.clone(), &wrong_pin, &signatures).unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::RootPinMismatch
        );
        let pin = CoordinatorReleaseKeyPolicyRootPin {
            root_digest: candidate.root_digest().unwrap(),
            root_profile: ROOT_PROFILE.into(),
        };
        assert_eq!(
            bootstrap_policy_root(candidate, &pin, &signatures[..1]).unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::ThresholdSignatureCountMismatch
        );
    }

    #[test]
    fn root_and_policy_roles_must_be_disjoint() {
        let now = system_time_ms().unwrap();
        let shared = TestSigner::new("shared", 41);
        let bad = root(now, 1, std::slice::from_ref(&shared), std::slice::from_ref(&shared), 1, 1);
        assert_eq!(
            bad.validate().unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::CrossRoleKeyReuse
        );
    }

    #[test]
    fn rotation_requires_old_and_new_thresholds_and_preserves_floors() {
        let now = system_time_ms().unwrap();
        let old_roots = vec![TestSigner::new("or1", 51), TestSigner::new("or2", 52)];
        let old_policy = vec![TestSigner::new("op1", 61), TestSigner::new("op2", 62)];
        let current = bootstrap(root(now, 1, &old_roots, &old_policy, 2, 2), &old_roots);
        let new_roots = vec![TestSigner::new("nr1", 71), TestSigner::new("nr2", 72)];
        let new_policy = vec![TestSigner::new("np1", 81), TestSigner::new("np2", 82)];
        let next = root(now, 2, &new_roots, &new_policy, 2, 2);
        let message = root_rotation_signature_message(current.root(), &next).unwrap();
        let old_sigs = vec![old_roots[0].sign(&message), old_roots[1].sign(&message)];
        let new_sigs = vec![new_roots[0].sign(&message), new_roots[1].sign(&message)];
        assert_eq!(
            rotate_policy_root(&current, next.clone(), &old_sigs, &new_sigs)
                .unwrap()
                .root()
                .root_version,
            2
        );
        assert_eq!(
            rotate_policy_root(&current, next, &old_sigs, &new_sigs[..1]).unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::ThresholdSignatureCountMismatch
        );
        let weakened = root(now, 2, &new_roots, &new_policy, 1, 1);
        let weakened_message = root_rotation_signature_message(current.root(), &weakened).unwrap();
        let weakened_old = vec![old_roots[0].sign(&weakened_message), old_roots[1].sign(&weakened_message)];
        let weakened_new = vec![new_roots[0].sign(&weakened_message)];
        assert_eq!(
            rotate_policy_root(&current, weakened, &weakened_old, &weakened_new).unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::ThresholdFloorWeakened
        );
    }

    #[test]
    fn policy_heads_are_parent_linked_and_rotated_roots_cannot_restart_generation() {
        let now = system_time_ms().unwrap();
        let roots = vec![TestSigner::new("r1", 91), TestSigner::new("r2", 92)];
        let policy_signers = vec![TestSigner::new("p1", 101), TestSigner::new("p2", 102)];
        let qualified_root = bootstrap(root(now, 1, &roots, &policy_signers, 2, 2), &roots);
        let p1 = policy(now, 1);
        let h1 = CoordinatorReleaseKeyPolicyHead {
            protocol_version: PROTOCOL_VERSION.into(),
            root_version: 1,
            policy_digest: p1.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: 1,
            previous_head_digest: None,
            valid_from_ms: now.saturating_sub(1_000),
            valid_until_ms: now + 20_000,
        };
        let m1 = policy_head_signature_message(qualified_root.root(), &h1).unwrap();
        let s1 = vec![policy_signers[0].sign(&m1), policy_signers[1].sign(&m1)];
        let q1 = qualify_current_policy_head(&qualified_root, p1, h1, &s1, None).unwrap();

        let p2 = policy(now, 2);
        let h2 = CoordinatorReleaseKeyPolicyHead {
            protocol_version: PROTOCOL_VERSION.into(),
            root_version: 1,
            policy_digest: p2.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: 2,
            previous_head_digest: Some(q1.head_digest()),
            valid_from_ms: now.saturating_sub(1_000),
            valid_until_ms: now + 20_000,
        };
        let m2 = policy_head_signature_message(qualified_root.root(), &h2).unwrap();
        let s2 = vec![policy_signers[0].sign(&m2), policy_signers[1].sign(&m2)];
        let q2 = qualify_current_policy_head(&qualified_root, p2, h2, &s2, Some(&q1)).unwrap();
        assert_eq!(q2.head().policy_generation, 2);

        let new_roots = vec![TestSigner::new("nr1", 111), TestSigner::new("nr2", 112)];
        let new_policy_signers = vec![TestSigner::new("np1", 121), TestSigner::new("np2", 122)];
        let next_root = root(now, 2, &new_roots, &new_policy_signers, 2, 2);
        let rotation_message = root_rotation_signature_message(qualified_root.root(), &next_root).unwrap();
        let old_sigs = vec![roots[0].sign(&rotation_message), roots[1].sign(&rotation_message)];
        let new_sigs = vec![new_roots[0].sign(&rotation_message), new_roots[1].sign(&rotation_message)];
        let rotated = rotate_policy_root(&qualified_root, next_root, &old_sigs, &new_sigs).unwrap();
        let reset_policy = policy(now, 1);
        let reset_head = CoordinatorReleaseKeyPolicyHead {
            protocol_version: PROTOCOL_VERSION.into(),
            root_version: 2,
            policy_digest: reset_policy.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: 1,
            previous_head_digest: None,
            valid_from_ms: now.saturating_sub(1_000),
            valid_until_ms: now + 20_000,
        };
        let reset_message = policy_head_signature_message(rotated.root(), &reset_head).unwrap();
        let reset_sigs = vec![new_policy_signers[0].sign(&reset_message), new_policy_signers[1].sign(&reset_message)];
        assert_eq!(
            qualify_current_policy_head(&rotated, reset_policy, reset_head, &reset_sigs, None)
                .unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::InvalidPolicyHeadGenesis
        );
    }

    #[test]
    fn forged_policy_head_and_overwide_head_deny() {
        let now = system_time_ms().unwrap();
        let roots = vec![TestSigner::new("r1", 131), TestSigner::new("r2", 132)];
        let policies = vec![TestSigner::new("p1", 141), TestSigner::new("p2", 142)];
        let outsider = TestSigner::new("p1", 143);
        let qualified_root = bootstrap(root(now, 1, &roots, &policies, 2, 2), &roots);
        let semantic = policy(now, 1);
        let mut head = CoordinatorReleaseKeyPolicyHead {
            protocol_version: PROTOCOL_VERSION.into(),
            root_version: 1,
            policy_digest: semantic.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: 1,
            previous_head_digest: None,
            valid_from_ms: now.saturating_sub(1_000),
            valid_until_ms: now + 20_000,
        };
        let message = policy_head_signature_message(qualified_root.root(), &head).unwrap();
        let forged = vec![outsider.sign(&message), policies[1].sign(&message)];
        assert_eq!(
            qualify_current_policy_head(&qualified_root, semantic.clone(), head.clone(), &forged, None)
                .unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::Ed25519VerificationFailed
        );
        head.valid_until_ms = head.valid_from_ms + MAX_POLICY_HEAD_LIFETIME_MS + 1;
        assert_eq!(
            head.validate().unwrap_err(),
            CoordinatorReleaseKeyPolicyRootError::PolicyHeadLifetimeTooWide
        );
    }
}

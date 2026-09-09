// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure current signing-key authorization for candidate Mycelix coordinator releases.
//!
//! This crate deliberately sits BEFORE release signature authentication. A candidate
//! [`CoordinatorReleaseManifest`] commits a release-policy digest/profile, while an
//! independently current semantic key policy defines the exact authorized hybrid
//! Ed25519 + ML-DSA-65 keys. The local join binds one exact candidate-manifest digest
//! to one exact current policy before any signature can be accepted.
//!
//! Current-policy verifier provenance and actual hybrid signature verification remain
//! separate trust domains.

use mycelix_authority_coordinator_release::{CoordinatorReleaseManifest, MANIFEST_PROFILE};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-release-key-policy-v0.1";
pub const POLICY_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-v1-blake3-framed";
pub const KEY_PROFILE: &str =
    "mycelix-authority-coordinator-release-hybrid-key-v1-ed25519-32-ml-dsa-65-1952";
pub const HYBRID_SIGNATURE_SCHEME: &str = "ed25519+ml-dsa-65";
pub const CURRENT_POLICY_PROOF_PROTOCOL: &str =
    "mycelix-authority-coordinator-release-key-policy-currentness-proof-v0.1";
pub const CURRENT_POLICY_PROOF_PROFILE: &str =
    "mycelix-authority-coordinator-release-key-policy-currentness-proof-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-manifest-key-policy-qualification-v1-blake3-framed";
pub const QUALIFIED_KEY_PROFILE: &str =
    "mycelix-authority-coordinator-release-manifest-bound-hybrid-key-v1-blake3-framed";

pub const ED25519_PUBLIC_KEY_LEN: usize = 32;
pub const ML_DSA_65_PUBLIC_KEY_LEN: usize = 1952;
pub const MAX_CURRENTNESS_PROOF_REUSE_MS: u64 = 30_000;

const DOMAIN_POLICY: &[u8] = b"mycelix/authority/coordinator-release-key-policy/v1";
const DOMAIN_KEY: &[u8] = b"mycelix/authority/coordinator-release-hybrid-key/v1";
const DOMAIN_CURRENTNESS: &[u8] =
    b"mycelix/authority/coordinator-release-key-policy-currentness/v1";
const DOMAIN_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-release-manifest-key-policy-qualification/v1";
const DOMAIN_QUALIFIED_KEY: &[u8] =
    b"mycelix/authority/coordinator-release-manifest-bound-hybrid-key/v1";
const MAX_TEXT_BYTES: usize = 2048;
const MAX_KEYS: usize = 64;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizedHybridReleaseKey {
    pub key_id: String,
    pub ed25519_public_key: Vec<u8>,
    pub ml_dsa_65_public_key: Vec<u8>,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl AuthorizedHybridReleaseKey {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseKeyPolicyError> {
        validate_text(&self.key_id, "release signing key id")?;
        if self.ed25519_public_key.len() != ED25519_PUBLIC_KEY_LEN
            || self.ed25519_public_key.iter().all(|byte| *byte == 0)
        {
            return Err(CoordinatorReleaseKeyPolicyError::InvalidEd25519PublicKey);
        }
        if self.ml_dsa_65_public_key.len() != ML_DSA_65_PUBLIC_KEY_LEN
            || self.ml_dsa_65_public_key.iter().all(|byte| *byte == 0)
        {
            return Err(CoordinatorReleaseKeyPolicyError::InvalidMlDsa65PublicKey);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseKeyPolicyError::InvalidKeyWindow);
        }
        Ok(())
    }

    pub fn key_digest(&self) -> Result<[u8; 32], CoordinatorReleaseKeyPolicyError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_KEY);
        frame(&mut hasher, KEY_PROFILE.as_bytes());
        frame(&mut hasher, HYBRID_SIGNATURE_SCHEME.as_bytes());
        frame(&mut hasher, self.key_id.as_bytes());
        frame(&mut hasher, &self.ed25519_public_key);
        frame(&mut hasher, &self.ml_dsa_65_public_key);
        frame(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseKeyPolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub policy_generation: u64,
    pub release_authority_ref: String,
    pub authorized_keys: Vec<AuthorizedHybridReleaseKey>,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl CoordinatorReleaseKeyPolicy {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseKeyPolicyError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorReleaseKeyPolicyError::WrongProtocol);
        }
        validate_text(&self.policy_id, "release key policy id")?;
        validate_text(&self.release_authority_ref, "release authority ref")?;
        if self.policy_generation == 0 {
            return Err(CoordinatorReleaseKeyPolicyError::InvalidPolicyGeneration);
        }
        if self.authorized_keys.is_empty() || self.authorized_keys.len() > MAX_KEYS {
            return Err(CoordinatorReleaseKeyPolicyError::InvalidKeySetSize);
        }
        for key in &self.authorized_keys {
            key.validate()?;
        }
        let mut ids: Vec<&str> = self
            .authorized_keys
            .iter()
            .map(|key| key.key_id.as_str())
            .collect();
        ids.sort_unstable();
        if ids.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(CoordinatorReleaseKeyPolicyError::DuplicateKeyId);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseKeyPolicyError::InvalidPolicyWindow);
        }
        Ok(())
    }

    pub fn policy_digest(&self) -> Result<[u8; 32], CoordinatorReleaseKeyPolicyError> {
        self.validate()?;
        let mut keys: Vec<&AuthorizedHybridReleaseKey> = self.authorized_keys.iter().collect();
        keys.sort_by(|left, right| left.key_id.cmp(&right.key_id));
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_POLICY);
        frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame(&mut hasher, POLICY_PROFILE.as_bytes());
        frame(&mut hasher, HYBRID_SIGNATURE_SCHEME.as_bytes());
        frame(&mut hasher, self.policy_id.as_bytes());
        frame(&mut hasher, &self.policy_generation.to_le_bytes());
        frame(&mut hasher, self.release_authority_ref.as_bytes());
        for key in keys {
            frame(&mut hasher, &key.key_digest()?);
            frame(&mut hasher, KEY_PROFILE.as_bytes());
        }
        frame(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Evidence-shaped currentness proof for one exact release signing-key policy.
///
/// This remains deserializable because it crosses the future current-policy verifier
/// boundary. Pure validation proves shape/equality only, never verifier provenance.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCurrentCoordinatorReleaseKeyPolicyProof {
    pub protocol_version: String,
    pub policy_digest: [u8; 32],
    pub policy_profile: String,
    pub policy_generation: u64,
    pub source_ref: String,
    pub verifier_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedCurrentCoordinatorReleaseKeyPolicyProof {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), CoordinatorReleaseKeyPolicyError> {
        if self.protocol_version != CURRENT_POLICY_PROOF_PROTOCOL {
            return Err(CoordinatorReleaseKeyPolicyError::WrongCurrentnessProofProtocol);
        }
        if self.policy_profile != POLICY_PROFILE {
            return Err(CoordinatorReleaseKeyPolicyError::WrongPolicyProfile);
        }
        validate_digest(&self.policy_digest, "release key policy digest")?;
        if self.policy_generation == 0 {
            return Err(CoordinatorReleaseKeyPolicyError::InvalidPolicyGeneration);
        }
        validate_text(&self.source_ref, "release key policy source ref")?;
        validate_text(&self.verifier_ref, "release key policy verifier ref")?;
        validate_window(self.verified_at_ms, self.valid_until_ms, now_ms)?;
        let width = self
            .valid_until_ms
            .checked_sub(self.verified_at_ms)
            .ok_or(CoordinatorReleaseKeyPolicyError::InvalidCurrentnessWindow)?;
        if width > MAX_CURRENTNESS_PROOF_REUSE_MS {
            return Err(CoordinatorReleaseKeyPolicyError::CurrentnessLeaseTooWide);
        }
        Ok(())
    }

    pub fn proof_digest(&self) -> Result<[u8; 32], CoordinatorReleaseKeyPolicyError> {
        let shape_now = self
            .valid_until_ms
            .checked_sub(1)
            .ok_or(CoordinatorReleaseKeyPolicyError::InvalidCurrentnessWindow)?;
        self.validate_at(shape_now)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_CURRENTNESS);
        frame(&mut hasher, CURRENT_POLICY_PROOF_PROTOCOL.as_bytes());
        frame(&mut hasher, CURRENT_POLICY_PROOF_PROFILE.as_bytes());
        frame(&mut hasher, &self.policy_digest);
        frame(&mut hasher, self.policy_profile.as_bytes());
        frame(&mut hasher, &self.policy_generation.to_le_bytes());
        frame(&mut hasher, self.source_ref.as_bytes());
        frame(&mut hasher, self.verifier_ref.as_bytes());
        frame(&mut hasher, &self.verified_at_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Non-deserializable current key-policy qualification bound to one exact candidate
/// coordinator release manifest.
///
/// This object does not prove the candidate manifest is signed/authenticated. It
/// proves only that the manifest commits the exact independently current key policy
/// needed to decide which signing keys are eligible to authenticate that manifest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoordinatorReleaseManifestKeyPolicy {
    policy: CoordinatorReleaseKeyPolicy,
    manifest_digest: [u8; 32],
    manifest_profile: String,
    release_authority_ref: String,
    release_policy_digest: [u8; 32],
    release_policy_profile: String,
    currentness_proof_digest: [u8; 32],
    currentness_proof_profile: String,
    qualification_digest: [u8; 32],
    qualification_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoordinatorReleaseManifestKeyPolicy {
    pub fn policy(&self) -> &CoordinatorReleaseKeyPolicy {
        &self.policy
    }

    pub fn manifest_digest(&self) -> [u8; 32] {
        self.manifest_digest
    }

    pub fn manifest_profile(&self) -> &str {
        &self.manifest_profile
    }

    pub fn release_authority_ref(&self) -> &str {
        &self.release_authority_ref
    }

    pub fn release_policy_digest(&self) -> [u8; 32] {
        self.release_policy_digest
    }

    pub fn release_policy_profile(&self) -> &str {
        &self.release_policy_profile
    }

    pub fn currentness_proof_digest(&self) -> [u8; 32] {
        self.currentness_proof_digest
    }

    pub fn currentness_proof_profile(&self) -> &str {
        &self.currentness_proof_profile
    }

    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn qualify_key(
        &self,
        key_id: &str,
        now_ms: u64,
    ) -> Result<QualifiedCoordinatorReleaseSigningKey, CoordinatorReleaseKeyPolicyError> {
        if now_ms == 0 || self.verified_at_ms > now_ms || self.valid_until_ms <= now_ms {
            return Err(CoordinatorReleaseKeyPolicyError::QualifiedPolicyNotLive);
        }
        let key = self
            .policy
            .authorized_keys
            .iter()
            .find(|key| key.key_id == key_id)
            .ok_or(CoordinatorReleaseKeyPolicyError::UnauthorizedKey)?;
        key.validate()?;

        let verified_at_ms = self.verified_at_ms.max(key.valid_from_ms);
        let valid_until_ms = self.valid_until_ms.min(key.valid_until_ms);
        if verified_at_ms > now_ms || valid_until_ms <= now_ms || valid_until_ms <= verified_at_ms {
            return Err(CoordinatorReleaseKeyPolicyError::AuthorizedKeyNotLive);
        }

        let key_digest = key.key_digest()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_QUALIFIED_KEY);
        frame(&mut hasher, QUALIFIED_KEY_PROFILE.as_bytes());
        frame(&mut hasher, &self.manifest_digest);
        frame(&mut hasher, self.manifest_profile.as_bytes());
        frame(&mut hasher, &self.qualification_digest);
        frame(&mut hasher, self.qualification_profile.as_bytes());
        frame(&mut hasher, &key_digest);
        frame(&mut hasher, KEY_PROFILE.as_bytes());
        frame(&mut hasher, &verified_at_ms.to_le_bytes());
        frame(&mut hasher, &valid_until_ms.to_le_bytes());
        let qualification_digest = *hasher.finalize().as_bytes();

        Ok(QualifiedCoordinatorReleaseSigningKey {
            key: key.clone(),
            manifest_digest: self.manifest_digest,
            manifest_profile: self.manifest_profile.clone(),
            release_authority_ref: self.release_authority_ref.clone(),
            release_policy_digest: self.release_policy_digest,
            release_policy_profile: self.release_policy_profile.clone(),
            key_digest,
            key_profile: KEY_PROFILE.into(),
            policy_qualification_digest: self.qualification_digest,
            policy_qualification_profile: self.qualification_profile.clone(),
            qualification_digest,
            qualification_profile: QUALIFIED_KEY_PROFILE.into(),
            verified_at_ms,
            valid_until_ms,
        })
    }
}

/// Non-deserializable manifest-bound hybrid signing-key capability.
///
/// A future cryptographic verifier must recompute the candidate manifest digest and
/// require exact equality with [`Self::manifest_digest`] before using these key bytes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoordinatorReleaseSigningKey {
    key: AuthorizedHybridReleaseKey,
    manifest_digest: [u8; 32],
    manifest_profile: String,
    release_authority_ref: String,
    release_policy_digest: [u8; 32],
    release_policy_profile: String,
    key_digest: [u8; 32],
    key_profile: String,
    policy_qualification_digest: [u8; 32],
    policy_qualification_profile: String,
    qualification_digest: [u8; 32],
    qualification_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoordinatorReleaseSigningKey {
    pub fn key_id(&self) -> &str {
        &self.key.key_id
    }

    pub fn ed25519_public_key(&self) -> &[u8] {
        &self.key.ed25519_public_key
    }

    pub fn ml_dsa_65_public_key(&self) -> &[u8] {
        &self.key.ml_dsa_65_public_key
    }

    pub fn manifest_digest(&self) -> [u8; 32] {
        self.manifest_digest
    }

    pub fn manifest_profile(&self) -> &str {
        &self.manifest_profile
    }

    pub fn release_authority_ref(&self) -> &str {
        &self.release_authority_ref
    }

    pub fn release_policy_digest(&self) -> [u8; 32] {
        self.release_policy_digest
    }

    pub fn release_policy_profile(&self) -> &str {
        &self.release_policy_profile
    }

    pub fn key_digest(&self) -> [u8; 32] {
        self.key_digest
    }

    pub fn key_profile(&self) -> &str {
        &self.key_profile
    }

    pub fn policy_qualification_digest(&self) -> [u8; 32] {
        self.policy_qualification_digest
    }

    pub fn policy_qualification_profile(&self) -> &str {
        &self.policy_qualification_profile
    }

    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }
}

pub fn qualify_manifest_key_policy(
    manifest: &CoordinatorReleaseManifest,
    policy: CoordinatorReleaseKeyPolicy,
    currentness: &VerifiedCurrentCoordinatorReleaseKeyPolicyProof,
    now_ms: u64,
) -> Result<QualifiedCoordinatorReleaseManifestKeyPolicy, CoordinatorReleaseKeyPolicyError> {
    manifest
        .validate()
        .map_err(|error| CoordinatorReleaseKeyPolicyError::CandidateManifest(error.to_string()))?;
    policy.validate()?;
    currentness.validate_at(now_ms)?;

    let manifest_digest = manifest
        .manifest_digest()
        .map_err(|error| CoordinatorReleaseKeyPolicyError::CandidateManifest(error.to_string()))?;
    let policy_digest = policy.policy_digest()?;

    if manifest.release_policy_profile != POLICY_PROFILE
        || manifest.release_policy_digest != policy_digest
    {
        return Err(CoordinatorReleaseKeyPolicyError::ManifestPolicyMismatch);
    }
    if manifest.release_authority_ref != policy.release_authority_ref {
        return Err(CoordinatorReleaseKeyPolicyError::ReleaseAuthorityMismatch);
    }
    if currentness.policy_digest != policy_digest
        || currentness.policy_profile != POLICY_PROFILE
        || currentness.policy_generation != policy.policy_generation
    {
        return Err(CoordinatorReleaseKeyPolicyError::CurrentPolicyMismatch);
    }

    let verified_at_ms = currentness
        .verified_at_ms
        .max(policy.valid_from_ms)
        .max(manifest.valid_from_ms);
    let valid_until_ms = currentness
        .valid_until_ms
        .min(policy.valid_until_ms)
        .min(manifest.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms || valid_until_ms <= verified_at_ms {
        return Err(CoordinatorReleaseKeyPolicyError::EmptyQualificationWindow);
    }

    let currentness_proof_digest = currentness.proof_digest()?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &manifest_digest);
    frame(&mut hasher, MANIFEST_PROFILE.as_bytes());
    frame(&mut hasher, manifest.release_authority_ref.as_bytes());
    frame(&mut hasher, &manifest.release_policy_digest);
    frame(&mut hasher, manifest.release_policy_profile.as_bytes());
    frame(&mut hasher, &policy_digest);
    frame(&mut hasher, POLICY_PROFILE.as_bytes());
    frame(&mut hasher, &currentness_proof_digest);
    frame(&mut hasher, CURRENT_POLICY_PROOF_PROFILE.as_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    let qualification_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCoordinatorReleaseManifestKeyPolicy {
        policy,
        manifest_digest,
        manifest_profile: MANIFEST_PROFILE.into(),
        release_authority_ref: manifest.release_authority_ref.clone(),
        release_policy_digest: manifest.release_policy_digest,
        release_policy_profile: manifest.release_policy_profile.clone(),
        currentness_proof_digest,
        currentness_proof_profile: CURRENT_POLICY_PROOF_PROFILE.into(),
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn validate_digest(
    value: &[u8; 32],
    field: &'static str,
) -> Result<(), CoordinatorReleaseKeyPolicyError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorReleaseKeyPolicyError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), CoordinatorReleaseKeyPolicyError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(CoordinatorReleaseKeyPolicyError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn validate_window(
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), CoordinatorReleaseKeyPolicyError> {
    if now_ms == 0
        || verified_at_ms == 0
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
        || valid_until_ms <= verified_at_ms
    {
        Err(CoordinatorReleaseKeyPolicyError::InvalidCurrentnessWindow)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorReleaseKeyPolicyError {
    WrongProtocol,
    WrongCurrentnessProofProtocol,
    WrongPolicyProfile,
    CandidateManifest(String),
    InvalidDigest(&'static str),
    InvalidText(&'static str),
    InvalidPolicyGeneration,
    InvalidKeySetSize,
    DuplicateKeyId,
    InvalidEd25519PublicKey,
    InvalidMlDsa65PublicKey,
    InvalidKeyWindow,
    InvalidPolicyWindow,
    InvalidCurrentnessWindow,
    CurrentnessLeaseTooWide,
    ManifestPolicyMismatch,
    ReleaseAuthorityMismatch,
    CurrentPolicyMismatch,
    EmptyQualificationWindow,
    QualifiedPolicyNotLive,
    UnauthorizedKey,
    AuthorizedKeyNotLive,
}

impl fmt::Display for CoordinatorReleaseKeyPolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator release key-policy protocol"),
            Self::WrongCurrentnessProofProtocol => {
                write!(f, "wrong coordinator release key-policy currentness protocol")
            }
            Self::WrongPolicyProfile => write!(f, "wrong coordinator release key-policy profile"),
            Self::CandidateManifest(error) => write!(f, "invalid candidate release manifest: {error}"),
            Self::InvalidDigest(field) => write!(f, "invalid {field}"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::InvalidPolicyGeneration => {
                write!(f, "release key-policy generation must be non-zero")
            }
            Self::InvalidKeySetSize => {
                write!(f, "release key-policy must contain 1..={MAX_KEYS} keys")
            }
            Self::DuplicateKeyId => write!(f, "release key-policy contains duplicate key ids"),
            Self::InvalidEd25519PublicKey => {
                write!(f, "Ed25519 public key must be exactly 32 non-zero bytes")
            }
            Self::InvalidMlDsa65PublicKey => {
                write!(f, "ML-DSA-65 public key must be exactly 1952 non-zero bytes")
            }
            Self::InvalidKeyWindow => write!(f, "release signing-key validity window is invalid"),
            Self::InvalidPolicyWindow => write!(f, "release key-policy validity window is invalid"),
            Self::InvalidCurrentnessWindow => write!(
                f,
                "release key-policy currentness proof is stale, future-dated or inverted"
            ),
            Self::CurrentnessLeaseTooWide => write!(
                f,
                "release key-policy currentness proof exceeds the v0.1 reuse cap"
            ),
            Self::ManifestPolicyMismatch => write!(
                f,
                "candidate release manifest does not commit the exact signing-key policy"
            ),
            Self::ReleaseAuthorityMismatch => {
                write!(f, "signing-key policy names another release authority")
            }
            Self::CurrentPolicyMismatch => write!(
                f,
                "currentness proof names another signing-key policy or generation"
            ),
            Self::EmptyQualificationWindow => write!(
                f,
                "candidate-manifest signing-key policy has no live qualification window"
            ),
            Self::QualifiedPolicyNotLive => {
                write!(f, "qualified candidate-manifest signing-key policy is not live")
            }
            Self::UnauthorizedKey => write!(
                f,
                "requested release signing key is not authorized by current policy"
            ),
            Self::AuthorizedKeyNotLive => write!(f, "authorized release signing key is not live"),
        }
    }
}

impl std::error::Error for CoordinatorReleaseKeyPolicyError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_coordinator_deployment::CoordinatorCodeIdentity;
    use mycelix_authority_coordinator_release::PROTOCOL_VERSION as RELEASE_PROTOCOL_VERSION;

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn key(id: &str, byte: u8) -> AuthorizedHybridReleaseKey {
        AuthorizedHybridReleaseKey {
            key_id: id.into(),
            ed25519_public_key: vec![byte; ED25519_PUBLIC_KEY_LEN],
            ml_dsa_65_public_key: vec![byte; ML_DSA_65_PUBLIC_KEY_LEN],
            valid_from_ms: 100,
            valid_until_ms: 700,
        }
    }

    fn policy() -> CoordinatorReleaseKeyPolicy {
        CoordinatorReleaseKeyPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            policy_id: "coordinator-release-keys".into(),
            policy_generation: 3,
            release_authority_ref: "release-authority:1".into(),
            authorized_keys: vec![key("release-key-1", 7), key("release-key-2", 8)],
            valid_from_ms: 90,
            valid_until_ms: 800,
        }
    }

    fn manifest(policy: &CoordinatorReleaseKeyPolicy) -> CoordinatorReleaseManifest {
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
            valid_from_ms: 100,
            valid_until_ms: 1_000,
        }
    }

    fn currentness(
        policy: &CoordinatorReleaseKeyPolicy,
    ) -> VerifiedCurrentCoordinatorReleaseKeyPolicyProof {
        VerifiedCurrentCoordinatorReleaseKeyPolicyProof {
            protocol_version: CURRENT_POLICY_PROOF_PROTOCOL.into(),
            policy_digest: policy.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: policy.policy_generation,
            source_ref: "release-key-policy:head:3".into(),
            verifier_ref: "release-key-policy-verifier:1".into(),
            verified_at_ms: 170,
            valid_until_ms: 500,
        }
    }

    #[test]
    fn candidate_manifest_and_current_policy_qualify_before_signature_authentication() {
        let policy = policy();
        let manifest = manifest(&policy);
        let proof = currentness(&policy);
        let qualified = qualify_manifest_key_policy(&manifest, policy, &proof, 200).unwrap();
        assert_eq!(qualified.manifest_digest(), manifest.manifest_digest().unwrap());
        assert_eq!(qualified.valid_until_ms(), 500);

        let key = qualified.qualify_key("release-key-1", 200).unwrap();
        assert_eq!(key.manifest_digest(), manifest.manifest_digest().unwrap());
        assert_eq!(key.manifest_profile(), MANIFEST_PROFILE);
        assert_eq!(key.ed25519_public_key().len(), ED25519_PUBLIC_KEY_LEN);
        assert_eq!(key.ml_dsa_65_public_key().len(), ML_DSA_65_PUBLIC_KEY_LEN);
        assert_eq!(key.valid_until_ms(), 500);
        assert_ne!(key.qualification_digest(), [0; 32]);
    }

    #[test]
    fn qualified_key_is_bound_to_one_exact_candidate_manifest() {
        let policy = policy();
        let first_manifest = manifest(&policy);
        let mut second_manifest = first_manifest.clone();
        second_manifest.release_version = 2;

        let proof = currentness(&policy);
        let first = qualify_manifest_key_policy(&first_manifest, policy.clone(), &proof, 200)
            .unwrap()
            .qualify_key("release-key-1", 200)
            .unwrap();
        let second = qualify_manifest_key_policy(&second_manifest, policy, &proof, 200)
            .unwrap()
            .qualify_key("release-key-1", 200)
            .unwrap();

        assert_ne!(first.manifest_digest(), second.manifest_digest());
        assert_ne!(first.qualification_digest(), second.qualification_digest());
    }

    #[test]
    fn manifest_must_commit_exact_current_policy_and_authority() {
        let policy = policy();
        let proof = currentness(&policy);

        let mut wrong_digest = manifest(&policy);
        wrong_digest.release_policy_digest[0] ^= 1;
        assert_eq!(
            qualify_manifest_key_policy(&wrong_digest, policy.clone(), &proof, 200).unwrap_err(),
            CoordinatorReleaseKeyPolicyError::ManifestPolicyMismatch
        );

        let mut wrong_profile = manifest(&policy);
        wrong_profile.release_policy_profile = "other-policy-profile".into();
        assert_eq!(
            qualify_manifest_key_policy(&wrong_profile, policy.clone(), &proof, 200).unwrap_err(),
            CoordinatorReleaseKeyPolicyError::ManifestPolicyMismatch
        );

        let mut wrong_authority = manifest(&policy);
        wrong_authority.release_authority_ref = "release-authority:other".into();
        assert_eq!(
            qualify_manifest_key_policy(&wrong_authority, policy, &proof, 200).unwrap_err(),
            CoordinatorReleaseKeyPolicyError::ReleaseAuthorityMismatch
        );
    }

    #[test]
    fn policy_digest_is_order_independent_but_key_identity_sensitive() {
        let first = policy();
        let mut reordered = first.clone();
        reordered.authorized_keys.reverse();
        assert_eq!(first.policy_digest().unwrap(), reordered.policy_digest().unwrap());

        let mut changed = first.clone();
        changed.authorized_keys[0].ed25519_public_key[0] ^= 1;
        assert_ne!(first.policy_digest().unwrap(), changed.policy_digest().unwrap());
    }

    #[test]
    fn wrong_current_policy_generation_or_overwide_lease_denies() {
        let policy = policy();
        let manifest = manifest(&policy);
        let mut proof = currentness(&policy);
        proof.policy_generation += 1;
        assert_eq!(
            qualify_manifest_key_policy(&manifest, policy.clone(), &proof, 200).unwrap_err(),
            CoordinatorReleaseKeyPolicyError::CurrentPolicyMismatch
        );

        let mut proof = currentness(&policy);
        proof.valid_until_ms = proof.verified_at_ms + MAX_CURRENTNESS_PROOF_REUSE_MS + 1;
        assert_eq!(
            qualify_manifest_key_policy(&manifest, policy, &proof, 200).unwrap_err(),
            CoordinatorReleaseKeyPolicyError::CurrentnessLeaseTooWide
        );
    }

    #[test]
    fn malformed_wire_key_or_duplicate_id_denies() {
        let mut bad = policy();
        bad.authorized_keys[0].ml_dsa_65_public_key.pop();
        assert_eq!(
            bad.validate().unwrap_err(),
            CoordinatorReleaseKeyPolicyError::InvalidMlDsa65PublicKey
        );

        let mut duplicate = policy();
        duplicate.authorized_keys[1].key_id = duplicate.authorized_keys[0].key_id.clone();
        assert_eq!(
            duplicate.validate().unwrap_err(),
            CoordinatorReleaseKeyPolicyError::DuplicateKeyId
        );
    }

    #[test]
    fn unauthorized_or_expired_key_denies() {
        let policy = policy();
        let manifest = manifest(&policy);
        let proof = currentness(&policy);
        let qualified = qualify_manifest_key_policy(&manifest, policy, &proof, 200).unwrap();
        assert_eq!(
            qualified.qualify_key("not-authorized", 200).unwrap_err(),
            CoordinatorReleaseKeyPolicyError::UnauthorizedKey
        );

        let mut policy = policy();
        policy.authorized_keys[0].valid_until_ms = 190;
        let manifest = manifest(&policy);
        let proof = currentness(&policy);
        let qualified = qualify_manifest_key_policy(&manifest, policy, &proof, 180).unwrap();
        assert_eq!(
            qualified.qualify_key("release-key-1", 200).unwrap_err(),
            CoordinatorReleaseKeyPolicyError::AuthorizedKeyNotLive
        );
    }

    #[test]
    fn invalid_candidate_manifest_denies_before_key_authorization() {
        let policy = policy();
        let mut manifest = manifest(&policy);
        manifest.release_version = 0;
        let proof = currentness(&policy);
        assert!(matches!(
            qualify_manifest_key_policy(&manifest, policy, &proof, 200),
            Err(CoordinatorReleaseKeyPolicyError::CandidateManifest(_))
        ));
    }
}

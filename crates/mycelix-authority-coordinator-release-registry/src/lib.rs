// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Offline-rooted complete current registry snapshots for coordinator releases.
//!
//! One independently rooted registry-head role authenticates a complete canonical
//! snapshot. Release status is then looked up locally inside the non-deserializable
//! qualified snapshot. #275 head/status receipts are private compatibility objects,
//! never caller-supplied positive authority.
//!
//! Snapshot lineage is monotone in-process: records may be added but never removed,
//! and terminal Withdrawn/Superseded records cannot become Active again. Durable
//! latest-snapshot persistence across restart remains a later native-state theorem.

use ed25519_dalek::{Signature as EdSignature, VerifyingKey as EdVerifyingKey};
use ml_dsa::signature::Verifier as MlVerifier;
use ml_dsa::{
    EncodedSignature, EncodedVerifyingKey, KeyInit as _, MlDsa65,
    Signature as MlSignature, VerifyingKey as MlVerifyingKey,
};
use mycelix_authority_coordinator_release::{
    MANIFEST_PROFILE, QualifiedCoordinatorReleaseRequirement,
};
use mycelix_authority_coordinator_release_currentness::{
    CoordinatorReleaseStatus, QualifiedCurrentCoordinatorRelease, REGISTRY_HEAD_PROFILE,
    STATUS_RECORD_PROFILE, VerifiedCoordinatorReleaseStatusAtHeadProof,
    VerifiedCurrentReleaseRegistryHeadProof, qualify_current_coordinator_release,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-release-registry-v0.1";
pub const ROOT_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-root-v1-blake3-framed";
pub const ROOT_PIN_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-root-pin-v1-blake3";
pub const TRUST_KEY_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-trust-key-v1-ed25519-32-ml-dsa-65-1952";
pub const THRESHOLD_SIGNATURE_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-threshold-signature-v1-ed25519-64-ml-dsa-65-3309";
pub const ROOT_ROTATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-root-rotation-v1-blake3-framed";
pub const SNAPSHOT_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-snapshot-qualification-v1-blake3-framed";
pub const HYBRID_SIGNATURE_SCHEME: &str = "ed25519+ml-dsa-65";
pub const ED25519_PUBLIC_KEY_LEN: usize = 32;
pub const ED25519_SIGNATURE_LEN: usize = 64;
pub const ML_DSA_65_PUBLIC_KEY_LEN: usize = 1952;
pub const ML_DSA_65_SIGNATURE_LEN: usize = 3309;
pub const MAX_REGISTRY_HEAD_LIFETIME_MS: u64 = 30_000;
pub const MAX_REGISTRY_RECORDS: usize = 65_536;
pub const MAX_TRUST_KEYS_PER_ROLE: usize = 16;

const DOMAIN_TRUST_KEY: &[u8] =
    b"mycelix/authority/coordinator-release-registry-trust-key/v1";
const DOMAIN_ROOT: &[u8] = b"mycelix/authority/coordinator-release-registry-root/v1";
const DOMAIN_ROOT_BOOTSTRAP: &[u8] =
    b"mycelix/authority/coordinator-release-registry-root-bootstrap/v1";
const DOMAIN_ROOT_ROTATION: &[u8] =
    b"mycelix/authority/coordinator-release-registry-root-rotation/v1";
const DOMAIN_ROOT_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-release-registry-root-qualification/v1";
const DOMAIN_RECORD: &[u8] = b"mycelix/authority/coordinator-release-status-record/v1";
const DOMAIN_SNAPSHOT: &[u8] = b"mycelix/authority/coordinator-release-registry-head/v1";
const DOMAIN_SNAPSHOT_SIGNATURE: &[u8] =
    b"mycelix/authority/coordinator-release-registry-head-signature/v1";
const DOMAIN_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-release-registry-snapshot-qualification/v1";
const DOMAIN_SIGNER_SET: &[u8] =
    b"mycelix/authority/coordinator-release-registry-signer-set/v1";
const MAX_TEXT_BYTES: usize = 2048;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HybridRegistryTrustKey {
    pub key_id: String,
    pub ed25519_public_key: Vec<u8>,
    pub ml_dsa_65_public_key: Vec<u8>,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl HybridRegistryTrustKey {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseRegistryError> {
        validate_text(&self.key_id, "registry trust key id")?;
        if self.ed25519_public_key.len() != ED25519_PUBLIC_KEY_LEN
            || self.ed25519_public_key.iter().all(|byte| *byte == 0)
        {
            return Err(CoordinatorReleaseRegistryError::InvalidEd25519PublicKey);
        }
        if self.ml_dsa_65_public_key.len() != ML_DSA_65_PUBLIC_KEY_LEN
            || self.ml_dsa_65_public_key.iter().all(|byte| *byte == 0)
        {
            return Err(CoordinatorReleaseRegistryError::InvalidMlDsa65PublicKey);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseRegistryError::InvalidTrustKeyWindow);
        }
        Ok(())
    }

    fn digest(&self) -> Result<[u8; 32], CoordinatorReleaseRegistryError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_TRUST_KEY);
        frame(&mut h, TRUST_KEY_PROFILE.as_bytes());
        frame(&mut h, HYBRID_SIGNATURE_SCHEME.as_bytes());
        frame(&mut h, self.key_id.as_bytes());
        frame(&mut h, &self.ed25519_public_key);
        frame(&mut h, &self.ml_dsa_65_public_key);
        frame(&mut h, &self.valid_from_ms.to_le_bytes());
        frame(&mut h, &self.valid_until_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }

    fn live_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HybridRegistryThresholdSignature {
    pub signature_profile: String,
    pub key_id: String,
    pub ed25519_signature: Vec<u8>,
    pub ml_dsa_65_signature: Vec<u8>,
}

impl HybridRegistryThresholdSignature {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseRegistryError> {
        if self.signature_profile != THRESHOLD_SIGNATURE_PROFILE {
            return Err(CoordinatorReleaseRegistryError::WrongThresholdSignatureProfile);
        }
        validate_text(&self.key_id, "registry threshold signer id")?;
        if self.ed25519_signature.len() != ED25519_SIGNATURE_LEN {
            return Err(CoordinatorReleaseRegistryError::InvalidEd25519SignatureLength);
        }
        if self.ml_dsa_65_signature.len() != ML_DSA_65_SIGNATURE_LEN {
            return Err(CoordinatorReleaseRegistryError::InvalidMlDsa65SignatureLength);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseRegistryTrustRoot {
    pub protocol_version: String,
    pub root_id: String,
    pub root_version: u64,
    pub release_authority_ref: String,
    pub registry_id: String,
    pub root_keys: Vec<HybridRegistryTrustKey>,
    pub root_threshold: u16,
    pub registry_head_keys: Vec<HybridRegistryTrustKey>,
    pub registry_head_threshold: u16,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl CoordinatorReleaseRegistryTrustRoot {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseRegistryError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorReleaseRegistryError::WrongProtocol);
        }
        validate_text(&self.root_id, "registry root id")?;
        validate_text(&self.release_authority_ref, "release authority ref")?;
        validate_text(&self.registry_id, "release registry id")?;
        if self.root_version == 0 {
            return Err(CoordinatorReleaseRegistryError::InvalidRootVersion);
        }
        validate_role_keys(&self.root_keys, self.root_threshold)?;
        validate_role_keys(&self.registry_head_keys, self.registry_head_threshold)?;
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseRegistryError::InvalidRootWindow);
        }
        let mut ids = BTreeSet::new();
        let mut materials = BTreeSet::new();
        for key in self.root_keys.iter().chain(self.registry_head_keys.iter()) {
            if !ids.insert(key.key_id.clone()) || !materials.insert(key_material_digest(key)) {
                return Err(CoordinatorReleaseRegistryError::CrossRoleKeyReuse);
            }
            if key.valid_until_ms <= self.valid_from_ms || key.valid_from_ms >= self.valid_until_ms {
                return Err(CoordinatorReleaseRegistryError::TrustKeyOutsideRootWindow);
            }
        }
        Ok(())
    }

    pub fn root_digest(&self) -> Result<[u8; 32], CoordinatorReleaseRegistryError> {
        self.validate()?;
        let mut root_keys: Vec<_> = self.root_keys.iter().collect();
        root_keys.sort_by(|left, right| left.key_id.cmp(&right.key_id));
        let mut head_keys: Vec<_> = self.registry_head_keys.iter().collect();
        head_keys.sort_by(|left, right| left.key_id.cmp(&right.key_id));
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_ROOT);
        frame(&mut h, PROTOCOL_VERSION.as_bytes());
        frame(&mut h, ROOT_PROFILE.as_bytes());
        frame(&mut h, HYBRID_SIGNATURE_SCHEME.as_bytes());
        frame(&mut h, self.root_id.as_bytes());
        frame(&mut h, &self.root_version.to_le_bytes());
        frame(&mut h, self.release_authority_ref.as_bytes());
        frame(&mut h, self.registry_id.as_bytes());
        frame(&mut h, &self.root_threshold.to_le_bytes());
        for key in root_keys {
            frame(&mut h, &key.digest()?);
        }
        frame(&mut h, &self.registry_head_threshold.to_le_bytes());
        for key in head_keys {
            frame(&mut h, &key.digest()?);
        }
        frame(&mut h, &self.valid_from_ms.to_le_bytes());
        frame(&mut h, &self.valid_until_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }

    fn live_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseRegistryRootPin {
    pub root_digest: [u8; 32],
    pub root_profile: String,
}

impl CoordinatorReleaseRegistryRootPin {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseRegistryError> {
        validate_digest(&self.root_digest, "registry root pin digest")?;
        if self.root_profile != ROOT_PROFILE {
            return Err(CoordinatorReleaseRegistryError::WrongRootProfile);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoordinatorReleaseRegistryRoot {
    root: CoordinatorReleaseRegistryTrustRoot,
    root_digest: [u8; 32],
    root_lineage_digest: [u8; 32],
    root_threshold_floor: u16,
    registry_head_threshold_floor: u16,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoordinatorReleaseRegistryRoot {
    pub fn root(&self) -> &CoordinatorReleaseRegistryTrustRoot { &self.root }
    pub fn root_digest(&self) -> [u8; 32] { self.root_digest }
    pub fn root_lineage_digest(&self) -> [u8; 32] { self.root_lineage_digest }
    pub fn verification_ref(&self) -> &str { &self.verification_ref }
    pub fn verified_at_ms(&self) -> u64 { self.verified_at_ms }
    pub fn valid_until_ms(&self) -> u64 { self.valid_until_ms }

    fn assert_live_at(&self, now_ms: u64) -> Result<(), CoordinatorReleaseRegistryError> {
        if self.verified_at_ms > now_ms || self.valid_until_ms <= now_ms || !self.root.live_at(now_ms) {
            Err(CoordinatorReleaseRegistryError::QualifiedRootNotLive)
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseRegistryRecord {
    pub manifest_digest: [u8; 32],
    pub manifest_profile: String,
    pub status: CoordinatorReleaseStatus,
    pub status_effective_at_ms: u64,
    pub status_ref: String,
}

impl CoordinatorReleaseRegistryRecord {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseRegistryError> {
        validate_digest(&self.manifest_digest, "registry record manifest digest")?;
        if self.manifest_profile != MANIFEST_PROFILE {
            return Err(CoordinatorReleaseRegistryError::WrongManifestProfile);
        }
        if self.status_effective_at_ms == 0 {
            return Err(CoordinatorReleaseRegistryError::InvalidStatusEffectiveTime);
        }
        validate_text(&self.status_ref, "registry status ref")?;
        Ok(())
    }

    pub fn status_record_digest(&self) -> Result<[u8; 32], CoordinatorReleaseRegistryError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_RECORD);
        frame(&mut h, STATUS_RECORD_PROFILE.as_bytes());
        frame(&mut h, &self.manifest_digest);
        frame(&mut h, self.manifest_profile.as_bytes());
        frame(&mut h, &[status_code(self.status)]);
        frame(&mut h, &self.status_effective_at_ms.to_le_bytes());
        frame(&mut h, self.status_ref.as_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseRegistrySnapshot {
    pub protocol_version: String,
    pub root_version: u64,
    pub registry_id: String,
    pub release_authority_ref: String,
    pub release_policy_digest: [u8; 32],
    pub release_policy_profile: String,
    pub registry_generation: u64,
    pub previous_registry_head_digest: Option<[u8; 32]>,
    pub records: Vec<CoordinatorReleaseRegistryRecord>,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl CoordinatorReleaseRegistrySnapshot {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseRegistryError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorReleaseRegistryError::WrongProtocol);
        }
        if self.root_version == 0 || self.registry_generation == 0 {
            return Err(CoordinatorReleaseRegistryError::InvalidRegistryGeneration);
        }
        validate_text(&self.registry_id, "release registry id")?;
        validate_text(&self.release_authority_ref, "release authority ref")?;
        validate_digest(&self.release_policy_digest, "release policy digest")?;
        validate_text(&self.release_policy_profile, "release policy profile")?;
        if let Some(previous) = self.previous_registry_head_digest {
            validate_digest(&previous, "previous registry head digest")?;
        }
        if self.records.len() > MAX_REGISTRY_RECORDS {
            return Err(CoordinatorReleaseRegistryError::TooManyRegistryRecords);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseRegistryError::InvalidSnapshotWindow);
        }
        let width = self
            .valid_until_ms
            .checked_sub(self.valid_from_ms)
            .ok_or(CoordinatorReleaseRegistryError::InvalidSnapshotWindow)?;
        if width > MAX_REGISTRY_HEAD_LIFETIME_MS {
            return Err(CoordinatorReleaseRegistryError::SnapshotLifetimeTooWide);
        }
        let mut previous_manifest: Option<[u8; 32]> = None;
        for record in &self.records {
            record.validate()?;
            if record.status_effective_at_ms > self.valid_from_ms {
                return Err(CoordinatorReleaseRegistryError::FutureStatusRecord);
            }
            if previous_manifest.is_some_and(|previous| previous >= record.manifest_digest) {
                return Err(CoordinatorReleaseRegistryError::RegistryRecordsNotCanonical);
            }
            previous_manifest = Some(record.manifest_digest);
        }
        Ok(())
    }

    pub fn head_digest(&self) -> Result<[u8; 32], CoordinatorReleaseRegistryError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_SNAPSHOT);
        frame(&mut h, REGISTRY_HEAD_PROFILE.as_bytes());
        frame(&mut h, &self.root_version.to_le_bytes());
        frame(&mut h, self.registry_id.as_bytes());
        frame(&mut h, self.release_authority_ref.as_bytes());
        frame(&mut h, &self.release_policy_digest);
        frame(&mut h, self.release_policy_profile.as_bytes());
        frame(&mut h, &self.registry_generation.to_le_bytes());
        match self.previous_registry_head_digest {
            Some(previous) => {
                frame(&mut h, b"previous");
                frame(&mut h, &previous);
            }
            None => frame(&mut h, b"genesis"),
        }
        frame(&mut h, &(self.records.len() as u64).to_le_bytes());
        for record in &self.records {
            frame(&mut h, &record.status_record_digest()?);
        }
        frame(&mut h, &self.valid_from_ms.to_le_bytes());
        frame(&mut h, &self.valid_until_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }

    fn record(&self, manifest_digest: [u8; 32]) -> Option<&CoordinatorReleaseRegistryRecord> {
        self.records
            .binary_search_by_key(&manifest_digest, |record| record.manifest_digest)
            .ok()
            .and_then(|index| self.records.get(index))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCurrentCoordinatorReleaseRegistrySnapshot {
    root_digest: [u8; 32],
    root_lineage_digest: [u8; 32],
    snapshot: CoordinatorReleaseRegistrySnapshot,
    registry_head_digest: [u8; 32],
    signer_set_digest: [u8; 32],
    qualification_digest: [u8; 32],
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentCoordinatorReleaseRegistrySnapshot {
    pub fn snapshot(&self) -> &CoordinatorReleaseRegistrySnapshot { &self.snapshot }
    pub fn registry_generation(&self) -> u64 { self.snapshot.registry_generation }
    pub fn registry_head_digest(&self) -> [u8; 32] { self.registry_head_digest }
    pub fn root_lineage_digest(&self) -> [u8; 32] { self.root_lineage_digest }
    pub fn qualification_digest(&self) -> [u8; 32] { self.qualification_digest }
    pub fn verification_ref(&self) -> &str { &self.verification_ref }
    pub fn verified_at_ms(&self) -> u64 { self.verified_at_ms }
    pub fn valid_until_ms(&self) -> u64 { self.valid_until_ms }

    /// Status is derived from the complete qualified snapshot. The live public path
    /// accepts neither #275 current-head receipt bytes nor status-at-head receipt bytes.
    pub fn qualify_current_release(
        &self,
        release: QualifiedCoordinatorReleaseRequirement,
    ) -> Result<QualifiedCurrentCoordinatorRelease, CoordinatorReleaseRegistryError> {
        let now_ms = system_time_ms()?;
        if self.verified_at_ms > now_ms || self.valid_until_ms <= now_ms {
            return Err(CoordinatorReleaseRegistryError::QualifiedSnapshotNotLive);
        }
        if release.verified_at_ms() > now_ms || release.valid_until_ms() <= now_ms {
            return Err(CoordinatorReleaseRegistryError::AuthenticatedReleaseNotLive);
        }
        let manifest = release.manifest();
        if manifest.release_authority_ref != self.snapshot.release_authority_ref
            || manifest.release_policy_digest != self.snapshot.release_policy_digest
            || manifest.release_policy_profile != self.snapshot.release_policy_profile
        {
            return Err(CoordinatorReleaseRegistryError::ReleaseOutsideRegistryScope);
        }
        let record = self
            .snapshot
            .record(release.manifest_digest())
            .ok_or(CoordinatorReleaseRegistryError::ReleaseMissingFromCompleteSnapshot)?;

        let head_proof = VerifiedCurrentReleaseRegistryHeadProof {
            protocol_version: mycelix_authority_coordinator_release_currentness::PROTOCOL_VERSION.into(),
            release_policy_digest: self.snapshot.release_policy_digest,
            release_policy_profile: self.snapshot.release_policy_profile.clone(),
            registry_generation: self.snapshot.registry_generation,
            registry_head_digest: self.registry_head_digest,
            registry_head_profile: REGISTRY_HEAD_PROFILE.into(),
            source_ref: format!(
                "complete-registry-head-blake3:{}",
                encode_hex(&self.registry_head_digest)
            ),
            verifier_ref: format!("{}:head", self.verification_ref),
            verified_at_ms: self.verified_at_ms,
            valid_until_ms: self.valid_until_ms,
        };
        let status_proof = VerifiedCoordinatorReleaseStatusAtHeadProof {
            protocol_version: mycelix_authority_coordinator_release_currentness::PROTOCOL_VERSION.into(),
            manifest_digest: record.manifest_digest,
            manifest_profile: MANIFEST_PROFILE.into(),
            release_policy_digest: self.snapshot.release_policy_digest,
            release_policy_profile: self.snapshot.release_policy_profile.clone(),
            registry_generation: self.snapshot.registry_generation,
            registry_head_digest: self.registry_head_digest,
            registry_head_profile: REGISTRY_HEAD_PROFILE.into(),
            status: record.status,
            status_record_digest: record.status_record_digest()?,
            status_record_profile: STATUS_RECORD_PROFILE.into(),
            verifier_ref: format!("{}:local-status", self.verification_ref),
            verified_at_ms: self.verified_at_ms,
            valid_until_ms: self.valid_until_ms,
        };
        qualify_current_coordinator_release(release, &head_proof, &status_proof, now_ms)
            .map_err(|error| CoordinatorReleaseRegistryError::Currentness(error.to_string()))
    }
}

pub fn root_bootstrap_signature_message(
    root: &CoordinatorReleaseRegistryTrustRoot,
) -> Result<Vec<u8>, CoordinatorReleaseRegistryError> {
    let mut out = Vec::new();
    append_frame(&mut out, DOMAIN_ROOT_BOOTSTRAP);
    append_frame(&mut out, ROOT_PROFILE.as_bytes());
    append_frame(&mut out, &root.root_digest()?);
    Ok(out)
}

pub fn root_rotation_signature_message(
    old_root: &CoordinatorReleaseRegistryTrustRoot,
    new_root: &CoordinatorReleaseRegistryTrustRoot,
) -> Result<Vec<u8>, CoordinatorReleaseRegistryError> {
    old_root.validate()?;
    new_root.validate()?;
    let mut out = Vec::new();
    append_frame(&mut out, DOMAIN_ROOT_ROTATION);
    append_frame(&mut out, ROOT_ROTATION_PROFILE.as_bytes());
    append_frame(&mut out, &old_root.root_digest()?);
    append_frame(&mut out, &new_root.root_digest()?);
    append_frame(&mut out, &old_root.root_version.to_le_bytes());
    append_frame(&mut out, &new_root.root_version.to_le_bytes());
    Ok(out)
}

pub fn snapshot_signature_message(
    root: &CoordinatorReleaseRegistryTrustRoot,
    snapshot: &CoordinatorReleaseRegistrySnapshot,
) -> Result<Vec<u8>, CoordinatorReleaseRegistryError> {
    root.validate()?;
    snapshot.validate()?;
    let mut out = Vec::new();
    append_frame(&mut out, DOMAIN_SNAPSHOT_SIGNATURE);
    append_frame(&mut out, REGISTRY_HEAD_PROFILE.as_bytes());
    append_frame(&mut out, &root.root_digest()?);
    append_frame(&mut out, &snapshot.head_digest()?);
    Ok(out)
}

pub fn bootstrap_registry_root(
    root: CoordinatorReleaseRegistryTrustRoot,
    pin: &CoordinatorReleaseRegistryRootPin,
    signatures: &[HybridRegistryThresholdSignature],
) -> Result<QualifiedCoordinatorReleaseRegistryRoot, CoordinatorReleaseRegistryError> {
    root.validate()?;
    pin.validate()?;
    let digest = root.root_digest()?;
    if pin.root_digest != digest || pin.root_profile != ROOT_PROFILE {
        return Err(CoordinatorReleaseRegistryError::RootPinMismatch);
    }
    let started = system_time_ms()?;
    if !root.live_at(started) {
        return Err(CoordinatorReleaseRegistryError::RootNotLive);
    }
    let message = root_bootstrap_signature_message(&root)?;
    verify_exact_threshold(&root.root_keys, root.root_threshold, signatures, &message, started)?;
    let verified = system_time_ms()?;
    ensure_signers_live(&root.root_keys, signatures, verified)?;
    if !root.live_at(verified) {
        return Err(CoordinatorReleaseRegistryError::RootExpiredDuringVerification);
    }
    let root_valid_until = root.valid_until_ms;
    let signer_digest = signer_set_digest(signatures);
    let qdigest = qualified_root_digest(digest, digest, signer_digest, verified, root_valid_until);
    Ok(QualifiedCoordinatorReleaseRegistryRoot {
        root_threshold_floor: root.root_threshold,
        registry_head_threshold_floor: root.registry_head_threshold,
        verification_ref: format!("release-registry-root-blake3:{}", encode_hex(&qdigest)),
        root,
        root_digest: digest,
        root_lineage_digest: digest,
        verified_at_ms: verified,
        valid_until_ms: root_valid_until,
    })
}

pub fn rotate_registry_root(
    current: &QualifiedCoordinatorReleaseRegistryRoot,
    new_root: CoordinatorReleaseRegistryTrustRoot,
    old_signatures: &[HybridRegistryThresholdSignature],
    new_signatures: &[HybridRegistryThresholdSignature],
) -> Result<QualifiedCoordinatorReleaseRegistryRoot, CoordinatorReleaseRegistryError> {
    let started = system_time_ms()?;
    current.assert_live_at(started)?;
    new_root.validate()?;
    if new_root.root_id != current.root.root_id
        || new_root.release_authority_ref != current.root.release_authority_ref
        || new_root.registry_id != current.root.registry_id
    {
        return Err(CoordinatorReleaseRegistryError::RootScopeChanged);
    }
    let expected = current
        .root
        .root_version
        .checked_add(1)
        .ok_or(CoordinatorReleaseRegistryError::RootVersionOverflow)?;
    if new_root.root_version != expected {
        return Err(CoordinatorReleaseRegistryError::InvalidRootRotationVersion);
    }
    if new_root.root_threshold < current.root_threshold_floor
        || new_root.registry_head_threshold < current.registry_head_threshold_floor
    {
        return Err(CoordinatorReleaseRegistryError::ThresholdFloorWeakened);
    }
    let message = root_rotation_signature_message(&current.root, &new_root)?;
    verify_exact_threshold(
        &current.root.root_keys,
        current.root.root_threshold,
        old_signatures,
        &message,
        started,
    )?;
    verify_exact_threshold(
        &new_root.root_keys,
        new_root.root_threshold,
        new_signatures,
        &message,
        started,
    )?;
    let verified = system_time_ms()?;
    current.assert_live_at(verified)?;
    ensure_signers_live(&current.root.root_keys, old_signatures, verified)?;
    ensure_signers_live(&new_root.root_keys, new_signatures, verified)?;
    if !new_root.live_at(verified) {
        return Err(CoordinatorReleaseRegistryError::RootExpiredDuringVerification);
    }
    let digest = new_root.root_digest()?;
    let new_valid_until = new_root.valid_until_ms;
    let signer_digest = combined_signer_set_digest(old_signatures, new_signatures);
    let qdigest = qualified_root_digest(
        digest,
        current.root_lineage_digest,
        signer_digest,
        verified,
        new_valid_until,
    );
    Ok(QualifiedCoordinatorReleaseRegistryRoot {
        root_threshold_floor: current.root_threshold_floor.max(new_root.root_threshold),
        registry_head_threshold_floor: current
            .registry_head_threshold_floor
            .max(new_root.registry_head_threshold),
        verification_ref: format!("release-registry-root-blake3:{}", encode_hex(&qdigest)),
        root: new_root,
        root_digest: digest,
        root_lineage_digest: current.root_lineage_digest,
        verified_at_ms: verified,
        valid_until_ms: new_valid_until,
    })
}

pub fn qualify_current_registry_snapshot(
    root: &QualifiedCoordinatorReleaseRegistryRoot,
    snapshot: CoordinatorReleaseRegistrySnapshot,
    signatures: &[HybridRegistryThresholdSignature],
    previous: Option<&QualifiedCurrentCoordinatorReleaseRegistrySnapshot>,
) -> Result<QualifiedCurrentCoordinatorReleaseRegistrySnapshot, CoordinatorReleaseRegistryError> {
    let started = system_time_ms()?;
    root.assert_live_at(started)?;
    snapshot.validate()?;
    if snapshot.root_version != root.root.root_version
        || snapshot.registry_id != root.root.registry_id
        || snapshot.release_authority_ref != root.root.release_authority_ref
    {
        return Err(CoordinatorReleaseRegistryError::SnapshotOutsideRootScope);
    }
    if started < snapshot.valid_from_ms || started >= snapshot.valid_until_ms {
        return Err(CoordinatorReleaseRegistryError::SnapshotNotLive);
    }
    let head_digest = snapshot.head_digest()?;
    match previous {
        None => {
            if root.root.root_version != 1
                || snapshot.registry_generation != 1
                || snapshot.previous_registry_head_digest.is_some()
            {
                return Err(CoordinatorReleaseRegistryError::InvalidSnapshotGenesis);
            }
        }
        Some(previous) => {
            if previous.root_lineage_digest != root.root_lineage_digest
                || previous.snapshot.registry_id != snapshot.registry_id
                || previous.snapshot.release_authority_ref != snapshot.release_authority_ref
                || previous.snapshot.release_policy_digest != snapshot.release_policy_digest
                || previous.snapshot.release_policy_profile != snapshot.release_policy_profile
            {
                return Err(CoordinatorReleaseRegistryError::SnapshotLineageMismatch);
            }
            if previous.registry_head_digest == head_digest {
                if previous.snapshot.registry_generation != snapshot.registry_generation {
                    return Err(CoordinatorReleaseRegistryError::SnapshotRevalidationMismatch);
                }
            } else {
                let expected = previous
                    .snapshot
                    .registry_generation
                    .checked_add(1)
                    .ok_or(CoordinatorReleaseRegistryError::RegistryGenerationOverflow)?;
                if snapshot.registry_generation != expected
                    || snapshot.previous_registry_head_digest != Some(previous.registry_head_digest)
                {
                    return Err(CoordinatorReleaseRegistryError::SnapshotPredecessorMismatch);
                }
                require_monotone_records(&previous.snapshot.records, &snapshot.records)?;
            }
        }
    }
    let message = snapshot_signature_message(&root.root, &snapshot)?;
    verify_exact_threshold(
        &root.root.registry_head_keys,
        root.root.registry_head_threshold,
        signatures,
        &message,
        started,
    )?;
    let verified = system_time_ms()?;
    root.assert_live_at(verified)?;
    ensure_signers_live(&root.root.registry_head_keys, signatures, verified)?;
    if verified < snapshot.valid_from_ms || verified >= snapshot.valid_until_ms {
        return Err(CoordinatorReleaseRegistryError::SnapshotExpiredDuringVerification);
    }
    let valid_until = root
        .valid_until_ms
        .min(snapshot.valid_until_ms)
        .min(signer_horizon(&root.root.registry_head_keys, signatures)?);
    if valid_until <= verified {
        return Err(CoordinatorReleaseRegistryError::EmptySnapshotWindow);
    }
    let signer_set_digest = signer_set_digest(signatures);
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, SNAPSHOT_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, &root.root_lineage_digest);
    frame(&mut h, &root.root_digest);
    frame(&mut h, ROOT_PROFILE.as_bytes());
    frame(&mut h, &head_digest);
    frame(&mut h, REGISTRY_HEAD_PROFILE.as_bytes());
    frame(&mut h, &signer_set_digest);
    frame(&mut h, &verified.to_le_bytes());
    frame(&mut h, &valid_until.to_le_bytes());
    let qualification_digest = *h.finalize().as_bytes();
    Ok(QualifiedCurrentCoordinatorReleaseRegistrySnapshot {
        root_digest: root.root_digest,
        root_lineage_digest: root.root_lineage_digest,
        snapshot,
        registry_head_digest: head_digest,
        signer_set_digest,
        qualification_digest,
        verification_ref: format!(
            "complete-release-registry-blake3:{}",
            encode_hex(&qualification_digest)
        ),
        verified_at_ms: verified,
        valid_until_ms: valid_until,
    })
}

fn require_monotone_records(
    previous: &[CoordinatorReleaseRegistryRecord],
    next: &[CoordinatorReleaseRegistryRecord],
) -> Result<(), CoordinatorReleaseRegistryError> {
    let next_map: BTreeMap<[u8; 32], &CoordinatorReleaseRegistryRecord> =
        next.iter().map(|record| (record.manifest_digest, record)).collect();
    for old in previous {
        let new = next_map
            .get(&old.manifest_digest)
            .copied()
            .ok_or(CoordinatorReleaseRegistryError::RegistryRecordRemoved)?;
        if new.manifest_profile != old.manifest_profile
            || new.status_effective_at_ms < old.status_effective_at_ms
        {
            return Err(CoordinatorReleaseRegistryError::RegistryRecordHistoryRewritten);
        }
        if new.status == old.status {
            if new.status_effective_at_ms != old.status_effective_at_ms
                || new.status_ref != old.status_ref
            {
                return Err(CoordinatorReleaseRegistryError::RegistryRecordHistoryRewritten);
            }
        } else if old.status != CoordinatorReleaseStatus::Active
            || new.status == CoordinatorReleaseStatus::Active
            || new.status_effective_at_ms <= old.status_effective_at_ms
        {
            return Err(CoordinatorReleaseRegistryError::InvalidStatusTransition);
        }
    }
    Ok(())
}

fn validate_role_keys(
    keys: &[HybridRegistryTrustKey],
    threshold: u16,
) -> Result<(), CoordinatorReleaseRegistryError> {
    if keys.is_empty() || keys.len() > MAX_TRUST_KEYS_PER_ROLE {
        return Err(CoordinatorReleaseRegistryError::InvalidTrustKeySetSize);
    }
    if threshold == 0 || usize::from(threshold) > keys.len() {
        return Err(CoordinatorReleaseRegistryError::InvalidThreshold);
    }
    let mut ids = BTreeSet::new();
    let mut materials = BTreeSet::new();
    for key in keys {
        key.validate()?;
        if !ids.insert(key.key_id.clone()) || !materials.insert(key_material_digest(key)) {
            return Err(CoordinatorReleaseRegistryError::DuplicateTrustKey);
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
) -> Result<(), CoordinatorReleaseRegistryError> {
    if signatures.len() != usize::from(threshold) {
        return Err(CoordinatorReleaseRegistryError::ThresholdSignatureCountMismatch);
    }
    let mut ids = BTreeSet::new();
    for signature in signatures {
        signature.validate()?;
        if !ids.insert(signature.key_id.clone()) {
            return Err(CoordinatorReleaseRegistryError::DuplicateThresholdSigner);
        }
        let key = keys
            .iter()
            .find(|key| key.key_id == signature.key_id)
            .ok_or(CoordinatorReleaseRegistryError::UnauthorizedThresholdSigner)?;
        if !key.live_at(now_ms) {
            return Err(CoordinatorReleaseRegistryError::ThresholdSignerNotLive);
        }
        verify_hybrid(key, message, signature)?;
    }
    Ok(())
}

fn ensure_signers_live(
    keys: &[HybridRegistryTrustKey],
    signatures: &[HybridRegistryThresholdSignature],
    now_ms: u64,
) -> Result<(), CoordinatorReleaseRegistryError> {
    for signature in signatures {
        let key = keys
            .iter()
            .find(|key| key.key_id == signature.key_id)
            .ok_or(CoordinatorReleaseRegistryError::UnauthorizedThresholdSigner)?;
        if !key.live_at(now_ms) {
            return Err(CoordinatorReleaseRegistryError::ThresholdSignerExpiredDuringVerification);
        }
    }
    Ok(())
}

fn signer_horizon(
    keys: &[HybridRegistryTrustKey],
    signatures: &[HybridRegistryThresholdSignature],
) -> Result<u64, CoordinatorReleaseRegistryError> {
    signatures
        .iter()
        .map(|signature| {
            keys.iter()
                .find(|key| key.key_id == signature.key_id)
                .map(|key| key.valid_until_ms)
                .ok_or(CoordinatorReleaseRegistryError::UnauthorizedThresholdSigner)
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .min()
        .ok_or(CoordinatorReleaseRegistryError::ThresholdSignatureCountMismatch)
}

fn verify_hybrid(
    key: &HybridRegistryTrustKey,
    message: &[u8],
    signature: &HybridRegistryThresholdSignature,
) -> Result<(), CoordinatorReleaseRegistryError> {
    let ed_key: [u8; ED25519_PUBLIC_KEY_LEN] = key
        .ed25519_public_key
        .as_slice()
        .try_into()
        .map_err(|_| CoordinatorReleaseRegistryError::InvalidEd25519PublicKey)?;
    let ed_signature: [u8; ED25519_SIGNATURE_LEN] = signature
        .ed25519_signature
        .as_slice()
        .try_into()
        .map_err(|_| CoordinatorReleaseRegistryError::InvalidEd25519SignatureLength)?;
    let verifier = EdVerifyingKey::from_bytes(&ed_key)
        .map_err(|_| CoordinatorReleaseRegistryError::InvalidEd25519PublicKey)?;
    verifier
        .verify_strict(message, &EdSignature::from_bytes(&ed_signature))
        .map_err(|_| CoordinatorReleaseRegistryError::Ed25519VerificationFailed)?;

    let encoded_key = EncodedVerifyingKey::<MlDsa65>::try_from(key.ml_dsa_65_public_key.as_slice())
        .map_err(|_| CoordinatorReleaseRegistryError::InvalidMlDsa65PublicKey)?;
    let verifier = MlVerifyingKey::<MlDsa65>::decode(&encoded_key);
    let encoded_signature =
        EncodedSignature::<MlDsa65>::try_from(signature.ml_dsa_65_signature.as_slice())
            .map_err(|_| CoordinatorReleaseRegistryError::InvalidMlDsa65SignatureLength)?;
    let signature = MlSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or(CoordinatorReleaseRegistryError::InvalidMlDsa65Signature)?;
    verifier
        .verify(message, &signature)
        .map_err(|_| CoordinatorReleaseRegistryError::MlDsa65VerificationFailed)
}

fn signer_set_digest(signatures: &[HybridRegistryThresholdSignature]) -> [u8; 32] {
    let mut ids: Vec<_> = signatures
        .iter()
        .map(|signature| signature.key_id.as_str())
        .collect();
    ids.sort();
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_SIGNER_SET);
    for id in ids {
        frame(&mut h, id.as_bytes());
    }
    *h.finalize().as_bytes()
}

fn combined_signer_set_digest(
    left: &[HybridRegistryThresholdSignature],
    right: &[HybridRegistryThresholdSignature],
) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_SIGNER_SET);
    frame(&mut h, b"old");
    frame(&mut h, &signer_set_digest(left));
    frame(&mut h, b"new");
    frame(&mut h, &signer_set_digest(right));
    *h.finalize().as_bytes()
}

fn qualified_root_digest(
    root: [u8; 32],
    lineage: [u8; 32],
    signers: [u8; 32],
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_ROOT_QUALIFICATION);
    frame(&mut h, ROOT_PROFILE.as_bytes());
    frame(&mut h, &root);
    frame(&mut h, &lineage);
    frame(&mut h, &signers);
    frame(&mut h, &verified_at_ms.to_le_bytes());
    frame(&mut h, &valid_until_ms.to_le_bytes());
    *h.finalize().as_bytes()
}

fn key_material_digest(key: &HybridRegistryTrustKey) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_TRUST_KEY);
    frame(&mut h, &key.ed25519_public_key);
    frame(&mut h, &key.ml_dsa_65_public_key);
    *h.finalize().as_bytes()
}

fn status_code(status: CoordinatorReleaseStatus) -> u8 {
    match status {
        CoordinatorReleaseStatus::Active => 1,
        CoordinatorReleaseStatus::Withdrawn => 2,
        CoordinatorReleaseStatus::Superseded => 3,
    }
}

fn system_time_ms() -> Result<u64, CoordinatorReleaseRegistryError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| CoordinatorReleaseRegistryError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis())
        .map_err(|_| CoordinatorReleaseRegistryError::ClockOverflow)
}

fn validate_text(
    value: &str,
    field: &'static str,
) -> Result<(), CoordinatorReleaseRegistryError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(CoordinatorReleaseRegistryError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn validate_digest(
    value: &[u8; 32],
    field: &'static str,
) -> Result<(), CoordinatorReleaseRegistryError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorReleaseRegistryError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn append_frame(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    out.extend_from_slice(bytes);
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorReleaseRegistryError {
    WrongProtocol,
    WrongRootProfile,
    WrongManifestProfile,
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
    RootExpiredDuringVerification,
    QualifiedRootNotLive,
    RootScopeChanged,
    RootVersionOverflow,
    InvalidRootRotationVersion,
    ThresholdFloorWeakened,
    ThresholdSignatureCountMismatch,
    DuplicateThresholdSigner,
    UnauthorizedThresholdSigner,
    ThresholdSignerNotLive,
    ThresholdSignerExpiredDuringVerification,
    InvalidRegistryGeneration,
    RegistryGenerationOverflow,
    TooManyRegistryRecords,
    InvalidStatusEffectiveTime,
    InvalidSnapshotWindow,
    SnapshotLifetimeTooWide,
    FutureStatusRecord,
    RegistryRecordsNotCanonical,
    SnapshotOutsideRootScope,
    SnapshotNotLive,
    InvalidSnapshotGenesis,
    SnapshotLineageMismatch,
    SnapshotRevalidationMismatch,
    SnapshotPredecessorMismatch,
    RegistryRecordRemoved,
    RegistryRecordHistoryRewritten,
    InvalidStatusTransition,
    SnapshotExpiredDuringVerification,
    EmptySnapshotWindow,
    QualifiedSnapshotNotLive,
    AuthenticatedReleaseNotLive,
    ReleaseOutsideRegistryScope,
    ReleaseMissingFromCompleteSnapshot,
    Currentness(String),
    ClockBeforeUnixEpoch,
    ClockOverflow,
}

impl fmt::Display for CoordinatorReleaseRegistryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for CoordinatorReleaseRegistryError {}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as EdSigner, SigningKey as EdSigningKey};
    use ml_dsa::signature::{Keypair as _, Signer as MlSigner};
    use ml_dsa::{Generate as _, KeyExport as _, SigningKey as MlSigningKey};
    use mycelix_authority_coordinator_deployment::CoordinatorCodeIdentity;
    use mycelix_authority_coordinator_release::{
        CoordinatorReleaseManifest, PROTOCOL_VERSION as RELEASE_PROTOCOL,
        SIGNATURE_PROOF_PROTOCOL, VerifiedCoordinatorReleaseSignatureProof,
        qualify_coordinator_release,
    };

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

        fn key(&self, now: u64) -> HybridRegistryTrustKey {
            HybridRegistryTrustKey {
                key_id: self.id.clone(),
                ed25519_public_key: self.ed.verifying_key().to_bytes().to_vec(),
                ml_dsa_65_public_key: self.ml.verifying_key().encode().as_slice().to_vec(),
                valid_from_ms: now.saturating_sub(5_000),
                valid_until_ms: now + 120_000,
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

    fn d(byte: u8) -> [u8; 32] { [byte; 32] }
    fn holo(byte: u8) -> Vec<u8> { vec![byte; 39] }

    fn root(
        now: u64,
        root_signers: &[&Signer],
        head_signers: &[&Signer],
        root_id: &str,
    ) -> CoordinatorReleaseRegistryTrustRoot {
        CoordinatorReleaseRegistryTrustRoot {
            protocol_version: PROTOCOL_VERSION.into(),
            root_id: root_id.into(),
            root_version: 1,
            release_authority_ref: "release-authority:offline".into(),
            registry_id: "coordinator-release-registry".into(),
            root_keys: root_signers.iter().map(|signer| signer.key(now)).collect(),
            root_threshold: 2,
            registry_head_keys: head_signers.iter().map(|signer| signer.key(now)).collect(),
            registry_head_threshold: 2,
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 120_000,
        }
    }

    fn bootstrap(
        candidate: CoordinatorReleaseRegistryTrustRoot,
        signers: &[&Signer],
    ) -> QualifiedCoordinatorReleaseRegistryRoot {
        let pin = CoordinatorReleaseRegistryRootPin {
            root_digest: candidate.root_digest().unwrap(),
            root_profile: ROOT_PROFILE.into(),
        };
        let message = root_bootstrap_signature_message(&candidate).unwrap();
        let signatures: Vec<_> = signers.iter().take(2).map(|signer| signer.sign(&message)).collect();
        bootstrap_registry_root(candidate, &pin, &signatures).unwrap()
    }

    fn manifest(now: u64, id: u8) -> CoordinatorReleaseManifest {
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
            valid_from_ms: now.saturating_sub(5_000),
            valid_until_ms: now + 120_000,
        }
    }

    fn authenticated(
        now: u64,
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
            verified_at_ms: now,
            valid_until_ms: now + 20_000,
        };
        qualify_coordinator_release(manifest, &proof, now).unwrap()
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
        now: u64,
        generation: u64,
        previous: Option<[u8; 32]>,
        records: Vec<CoordinatorReleaseRegistryRecord>,
    ) -> CoordinatorReleaseRegistrySnapshot {
        CoordinatorReleaseRegistrySnapshot {
            protocol_version: PROTOCOL_VERSION.into(),
            root_version: 1,
            registry_id: "coordinator-release-registry".into(),
            release_authority_ref: "release-authority:offline".into(),
            release_policy_digest: d(8),
            release_policy_profile: "coordinator-release-policy-v1".into(),
            registry_generation: generation,
            previous_registry_head_digest: previous,
            records,
            valid_from_ms: now.saturating_sub(1_000),
            valid_until_ms: now + 20_000,
        }
    }

    fn qualify_snapshot(
        root: &QualifiedCoordinatorReleaseRegistryRoot,
        snapshot: CoordinatorReleaseRegistrySnapshot,
        signers: &[&Signer],
        previous: Option<&QualifiedCurrentCoordinatorReleaseRegistrySnapshot>,
    ) -> QualifiedCurrentCoordinatorReleaseRegistrySnapshot {
        let message = snapshot_signature_message(root.root(), &snapshot).unwrap();
        let signatures: Vec<_> = signers.iter().take(2).map(|signer| signer.sign(&message)).collect();
        qualify_current_registry_snapshot(root, snapshot, &signatures, previous).unwrap()
    }

    #[test]
    fn complete_snapshot_locally_qualifies_active_release() {
        let now = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 1);
        let r2 = Signer::new("r2", 2);
        let h1 = Signer::new("h1", 11);
        let h2 = Signer::new("h2", 12);
        let qroot = bootstrap(root(now, &[&r1, &r2], &[&h1, &h2], "root-a"), &[&r1, &r2]);
        let manifest = manifest(now, 21);
        let release = authenticated(now, manifest.clone());
        let current = qualify_snapshot(
            &qroot,
            snapshot(
                now,
                1,
                None,
                vec![record(
                    manifest.manifest_digest().unwrap(),
                    CoordinatorReleaseStatus::Active,
                    now.saturating_sub(2_000),
                )],
            ),
            &[&h1, &h2],
            None,
        );
        let qualified = current.qualify_current_release(release).unwrap();
        assert_eq!(qualified.release().manifest_digest(), manifest.manifest_digest().unwrap());
    }

    #[test]
    fn missing_release_and_terminal_status_deny() {
        let now = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 21);
        let r2 = Signer::new("r2", 22);
        let h1 = Signer::new("h1", 31);
        let h2 = Signer::new("h2", 32);
        let qroot = bootstrap(root(now, &[&r1, &r2], &[&h1, &h2], "root-a"), &[&r1, &r2]);
        let manifest = manifest(now, 41);
        let empty = qualify_snapshot(&qroot, snapshot(now, 1, None, vec![]), &[&h1, &h2], None);
        assert_eq!(
            empty.qualify_current_release(authenticated(now, manifest.clone())).unwrap_err(),
            CoordinatorReleaseRegistryError::ReleaseMissingFromCompleteSnapshot
        );
        let withdrawn = qualify_snapshot(
            &qroot,
            snapshot(
                now,
                1,
                None,
                vec![record(
                    manifest.manifest_digest().unwrap(),
                    CoordinatorReleaseStatus::Withdrawn,
                    now.saturating_sub(2_000),
                )],
            ),
            &[&h1, &h2],
            None,
        );
        assert!(matches!(
            withdrawn.qualify_current_release(authenticated(now, manifest)),
            Err(CoordinatorReleaseRegistryError::Currentness(_))
        ));
    }

    #[test]
    fn successor_cannot_remove_record_or_resurrect_terminal_status() {
        let now = system_time_ms().unwrap();
        let r1 = Signer::new("r1", 41);
        let r2 = Signer::new("r2", 42);
        let h1 = Signer::new("h1", 51);
        let h2 = Signer::new("h2", 52);
        let qroot = bootstrap(root(now, &[&r1, &r2], &[&h1, &h2], "root-a"), &[&r1, &r2]);
        let first = qualify_snapshot(
            &qroot,
            snapshot(
                now,
                1,
                None,
                vec![record(d(61), CoordinatorReleaseStatus::Active, now.saturating_sub(3_000))],
            ),
            &[&h1, &h2],
            None,
        );
        let removed = snapshot(now, 2, Some(first.registry_head_digest()), vec![]);
        assert_eq!(
            qualify_current_registry_snapshot(&qroot, removed, &[], Some(&first)).unwrap_err(),
            CoordinatorReleaseRegistryError::RegistryRecordRemoved
        );
        let terminal = qualify_snapshot(
            &qroot,
            snapshot(
                now,
                2,
                Some(first.registry_head_digest()),
                vec![record(
                    d(61),
                    CoordinatorReleaseStatus::Withdrawn,
                    now.saturating_sub(1_500),
                )],
            ),
            &[&h1, &h2],
            Some(&first),
        );
        let resurrect = snapshot(
            now,
            3,
            Some(terminal.registry_head_digest()),
            vec![record(d(61), CoordinatorReleaseStatus::Active, now.saturating_sub(500))],
        );
        assert_eq!(
            qualify_current_registry_snapshot(&qroot, resurrect, &[], Some(&terminal)).unwrap_err(),
            CoordinatorReleaseRegistryError::InvalidStatusTransition
        );
    }

    #[test]
    fn predecessor_from_another_pinned_root_lineage_denies() {
        let now = system_time_ms().unwrap();
        let ar1 = Signer::new("ar1", 61);
        let ar2 = Signer::new("ar2", 62);
        let ah1 = Signer::new("ah1", 63);
        let ah2 = Signer::new("ah2", 64);
        let br1 = Signer::new("br1", 71);
        let br2 = Signer::new("br2", 72);
        let bh1 = Signer::new("bh1", 73);
        let bh2 = Signer::new("bh2", 74);
        let root_a = bootstrap(root(now, &[&ar1, &ar2], &[&ah1, &ah2], "root-a"), &[&ar1, &ar2]);
        let root_b = bootstrap(root(now, &[&br1, &br2], &[&bh1, &bh2], "root-b"), &[&br1, &br2]);
        let previous = qualify_snapshot(
            &root_a,
            snapshot(now, 1, None, vec![record(d(81), CoordinatorReleaseStatus::Active, now.saturating_sub(2_000))]),
            &[&ah1, &ah2],
            None,
        );
        let next = snapshot(
            now,
            2,
            Some(previous.registry_head_digest()),
            vec![record(d(81), CoordinatorReleaseStatus::Active, now.saturating_sub(2_000))],
        );
        assert_eq!(
            qualify_current_registry_snapshot(&root_b, next, &[], Some(&previous)).unwrap_err(),
            CoordinatorReleaseRegistryError::SnapshotLineageMismatch
        );
    }

    #[test]
    fn noncanonical_duplicate_or_overwide_snapshot_denies() {
        let now = system_time_ms().unwrap();
        let mut bad = snapshot(
            now,
            1,
            None,
            vec![
                record(d(3), CoordinatorReleaseStatus::Active, now.saturating_sub(2_000)),
                record(d(2), CoordinatorReleaseStatus::Active, now.saturating_sub(2_000)),
            ],
        );
        assert_eq!(bad.validate().unwrap_err(), CoordinatorReleaseRegistryError::RegistryRecordsNotCanonical);
        bad.records = vec![
            record(d(3), CoordinatorReleaseStatus::Active, now.saturating_sub(2_000)),
            record(d(3), CoordinatorReleaseStatus::Active, now.saturating_sub(2_000)),
        ];
        assert_eq!(bad.validate().unwrap_err(), CoordinatorReleaseRegistryError::RegistryRecordsNotCanonical);
        bad.records.clear();
        bad.valid_from_ms = now;
        bad.valid_until_ms = now + MAX_REGISTRY_HEAD_LIFETIME_MS + 1;
        assert_eq!(bad.validate().unwrap_err(), CoordinatorReleaseRegistryError::SnapshotLifetimeTooWide);
    }
}

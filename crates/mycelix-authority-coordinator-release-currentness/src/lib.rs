// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure currentness/withdrawal theorem for authenticated coordinator releases.
//!
//! A signed release is not deployable merely because its signature remains valid.
//! Currentness requires an independently verified current release-registry head and
//! an independently verified status proof for the exact manifest at that exact head.

use mycelix_authority_coordinator_release::{
    QualifiedCoordinatorReleaseRequirement, MANIFEST_PROFILE,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-release-currentness-v0.1";
pub const REGISTRY_HEAD_PROFILE: &str =
    "mycelix-authority-coordinator-release-registry-head-v1-blake3-framed";
pub const STATUS_RECORD_PROFILE: &str =
    "mycelix-authority-coordinator-release-status-record-v1-blake3-framed";
pub const STATUS_PROOF_PROFILE: &str =
    "mycelix-authority-coordinator-release-status-proof-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-currentness-qualification-v1-blake3-framed";

const DOMAIN_HEAD: &[u8] = b"mycelix/authority/coordinator-release-registry-head/v1";
const DOMAIN_STATUS: &[u8] = b"mycelix/authority/coordinator-release-status-proof/v1";
const DOMAIN_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-release-currentness-qualification/v1";
const MAX_TEXT_BYTES: usize = 2048;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CoordinatorReleaseStatus {
    Active,
    Withdrawn,
    Superseded,
}

/// Evidence-shaped receipt from the release-registry head verifier.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCurrentReleaseRegistryHeadProof {
    pub protocol_version: String,
    pub release_policy_digest: [u8; 32],
    pub release_policy_profile: String,
    pub registry_generation: u64,
    pub registry_head_digest: [u8; 32],
    pub registry_head_profile: String,
    pub source_ref: String,
    pub verifier_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedCurrentReleaseRegistryHeadProof {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), CoordinatorReleaseCurrentnessError> {
        require_protocol(&self.protocol_version)?;
        validate_digest(&self.release_policy_digest, "release policy digest")?;
        validate_digest(&self.registry_head_digest, "registry head digest")?;
        validate_text(&self.release_policy_profile, "release policy profile")?;
        if self.registry_head_profile != REGISTRY_HEAD_PROFILE {
            return Err(CoordinatorReleaseCurrentnessError::WrongRegistryHeadProfile);
        }
        validate_text(&self.source_ref, "registry source ref")?;
        validate_text(&self.verifier_ref, "registry-head verifier ref")?;
        if self.registry_generation == 0 {
            return Err(CoordinatorReleaseCurrentnessError::InvalidRegistryGeneration);
        }
        validate_window(self.verified_at_ms, self.valid_until_ms, now_ms)?;
        Ok(())
    }

    pub fn proof_digest(&self) -> Result<[u8; 32], CoordinatorReleaseCurrentnessError> {
        let shape_now = self
            .valid_until_ms
            .checked_sub(1)
            .ok_or(CoordinatorReleaseCurrentnessError::InvalidEvidenceWindow)?;
        self.validate_at(shape_now)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_HEAD);
        frame(&mut hasher, REGISTRY_HEAD_PROFILE.as_bytes());
        frame(&mut hasher, &self.release_policy_digest);
        frame(&mut hasher, self.release_policy_profile.as_bytes());
        frame(&mut hasher, &self.registry_generation.to_le_bytes());
        frame(&mut hasher, &self.registry_head_digest);
        frame(&mut hasher, self.registry_head_profile.as_bytes());
        frame(&mut hasher, self.source_ref.as_bytes());
        frame(&mut hasher, self.verifier_ref.as_bytes());
        frame(&mut hasher, &self.verified_at_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Evidence-shaped proof of one exact release's status under one exact registry head.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCoordinatorReleaseStatusAtHeadProof {
    pub protocol_version: String,
    pub manifest_digest: [u8; 32],
    pub manifest_profile: String,
    pub release_policy_digest: [u8; 32],
    pub release_policy_profile: String,
    pub registry_generation: u64,
    pub registry_head_digest: [u8; 32],
    pub registry_head_profile: String,
    pub status: CoordinatorReleaseStatus,
    pub status_record_digest: [u8; 32],
    pub status_record_profile: String,
    pub verifier_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedCoordinatorReleaseStatusAtHeadProof {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), CoordinatorReleaseCurrentnessError> {
        require_protocol(&self.protocol_version)?;
        validate_digest(&self.manifest_digest, "manifest digest")?;
        validate_digest(&self.release_policy_digest, "release policy digest")?;
        validate_digest(&self.registry_head_digest, "registry head digest")?;
        validate_digest(&self.status_record_digest, "status record digest")?;
        if self.manifest_profile != MANIFEST_PROFILE {
            return Err(CoordinatorReleaseCurrentnessError::WrongManifestProfile);
        }
        validate_text(&self.release_policy_profile, "release policy profile")?;
        if self.registry_head_profile != REGISTRY_HEAD_PROFILE {
            return Err(CoordinatorReleaseCurrentnessError::WrongRegistryHeadProfile);
        }
        if self.status_record_profile != STATUS_RECORD_PROFILE {
            return Err(CoordinatorReleaseCurrentnessError::WrongStatusRecordProfile);
        }
        validate_text(&self.verifier_ref, "release-status verifier ref")?;
        if self.registry_generation == 0 {
            return Err(CoordinatorReleaseCurrentnessError::InvalidRegistryGeneration);
        }
        validate_window(self.verified_at_ms, self.valid_until_ms, now_ms)?;
        Ok(())
    }

    pub fn proof_digest(&self) -> Result<[u8; 32], CoordinatorReleaseCurrentnessError> {
        let shape_now = self
            .valid_until_ms
            .checked_sub(1)
            .ok_or(CoordinatorReleaseCurrentnessError::InvalidEvidenceWindow)?;
        self.validate_at(shape_now)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_STATUS);
        frame(&mut hasher, STATUS_PROOF_PROFILE.as_bytes());
        frame(&mut hasher, &self.manifest_digest);
        frame(&mut hasher, self.manifest_profile.as_bytes());
        frame(&mut hasher, &self.release_policy_digest);
        frame(&mut hasher, self.release_policy_profile.as_bytes());
        frame(&mut hasher, &self.registry_generation.to_le_bytes());
        frame(&mut hasher, &self.registry_head_digest);
        frame(&mut hasher, self.registry_head_profile.as_bytes());
        frame(&mut hasher, &[status_code(self.status)]);
        frame(&mut hasher, &self.status_record_digest);
        frame(&mut hasher, self.status_record_profile.as_bytes());
        frame(&mut hasher, self.verifier_ref.as_bytes());
        frame(&mut hasher, &self.verified_at_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Non-deserializable proof that one authenticated release is `Active` under the
/// independently verified current release-registry head.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCurrentCoordinatorRelease {
    release: QualifiedCoordinatorReleaseRequirement,
    registry_generation: u64,
    registry_head_digest: [u8; 32],
    registry_head_profile: String,
    registry_head_proof_digest: [u8; 32],
    status_proof_digest: [u8; 32],
    qualification_digest: [u8; 32],
    qualification_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentCoordinatorRelease {
    pub fn release(&self) -> &QualifiedCoordinatorReleaseRequirement {
        &self.release
    }

    pub fn registry_generation(&self) -> u64 {
        self.registry_generation
    }

    pub fn registry_head_digest(&self) -> [u8; 32] {
        self.registry_head_digest
    }

    pub fn registry_head_profile(&self) -> &str {
        &self.registry_head_profile
    }

    pub fn registry_head_proof_digest(&self) -> [u8; 32] {
        self.registry_head_proof_digest
    }

    pub fn status_proof_digest(&self) -> [u8; 32] {
        self.status_proof_digest
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

pub fn qualify_current_coordinator_release(
    release: QualifiedCoordinatorReleaseRequirement,
    current_head: &VerifiedCurrentReleaseRegistryHeadProof,
    status_proof: &VerifiedCoordinatorReleaseStatusAtHeadProof,
    now_ms: u64,
) -> Result<QualifiedCurrentCoordinatorRelease, CoordinatorReleaseCurrentnessError> {
    current_head.validate_at(now_ms)?;
    status_proof.validate_at(now_ms)?;
    if release.verified_at_ms() > now_ms || release.valid_until_ms() <= now_ms {
        return Err(CoordinatorReleaseCurrentnessError::AuthenticatedReleaseNotLive);
    }

    let manifest = release.manifest();
    if current_head.release_policy_digest != manifest.release_policy_digest
        || current_head.release_policy_profile != manifest.release_policy_profile
    {
        return Err(CoordinatorReleaseCurrentnessError::ReleasePolicyMismatch);
    }
    if status_proof.manifest_digest != release.manifest_digest()
        || status_proof.manifest_profile != MANIFEST_PROFILE
    {
        return Err(CoordinatorReleaseCurrentnessError::ManifestMismatch);
    }
    if status_proof.release_policy_digest != current_head.release_policy_digest
        || status_proof.release_policy_profile != current_head.release_policy_profile
    {
        return Err(CoordinatorReleaseCurrentnessError::ReleasePolicyMismatch);
    }
    if status_proof.registry_generation != current_head.registry_generation
        || status_proof.registry_head_digest != current_head.registry_head_digest
        || status_proof.registry_head_profile != current_head.registry_head_profile
    {
        return Err(CoordinatorReleaseCurrentnessError::StatusForAnotherRegistryHead);
    }
    if status_proof.status != CoordinatorReleaseStatus::Active {
        return Err(CoordinatorReleaseCurrentnessError::ReleaseNotActive(
            status_proof.status,
        ));
    }

    let verified_at_ms = release
        .verified_at_ms()
        .max(current_head.verified_at_ms)
        .max(status_proof.verified_at_ms);
    let valid_until_ms = release
        .valid_until_ms()
        .min(current_head.valid_until_ms)
        .min(status_proof.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms || valid_until_ms <= verified_at_ms {
        return Err(CoordinatorReleaseCurrentnessError::EmptyQualificationWindow);
    }

    let registry_head_proof_digest = current_head.proof_digest()?;
    let status_proof_digest = status_proof.proof_digest()?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &release.qualification_digest());
    frame(&mut hasher, release.qualification_profile().as_bytes());
    frame(&mut hasher, &registry_head_proof_digest);
    frame(&mut hasher, REGISTRY_HEAD_PROFILE.as_bytes());
    frame(&mut hasher, &status_proof_digest);
    frame(&mut hasher, STATUS_PROOF_PROFILE.as_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    let qualification_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCurrentCoordinatorRelease {
        release,
        registry_generation: current_head.registry_generation,
        registry_head_digest: current_head.registry_head_digest,
        registry_head_profile: current_head.registry_head_profile.clone(),
        registry_head_proof_digest,
        status_proof_digest,
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn require_protocol(value: &str) -> Result<(), CoordinatorReleaseCurrentnessError> {
    if value == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(CoordinatorReleaseCurrentnessError::WrongProtocol)
    }
}

fn validate_digest(
    value: &[u8; 32],
    field: &'static str,
) -> Result<(), CoordinatorReleaseCurrentnessError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorReleaseCurrentnessError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), CoordinatorReleaseCurrentnessError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(CoordinatorReleaseCurrentnessError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn validate_window(
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), CoordinatorReleaseCurrentnessError> {
    if now_ms == 0
        || verified_at_ms == 0
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
        || valid_until_ms <= verified_at_ms
    {
        Err(CoordinatorReleaseCurrentnessError::InvalidEvidenceWindow)
    } else {
        Ok(())
    }
}

fn status_code(status: CoordinatorReleaseStatus) -> u8 {
    match status {
        CoordinatorReleaseStatus::Active => 1,
        CoordinatorReleaseStatus::Withdrawn => 2,
        CoordinatorReleaseStatus::Superseded => 3,
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorReleaseCurrentnessError {
    WrongProtocol,
    WrongManifestProfile,
    WrongRegistryHeadProfile,
    WrongStatusRecordProfile,
    InvalidDigest(&'static str),
    InvalidText(&'static str),
    InvalidRegistryGeneration,
    InvalidEvidenceWindow,
    AuthenticatedReleaseNotLive,
    ReleasePolicyMismatch,
    ManifestMismatch,
    StatusForAnotherRegistryHead,
    ReleaseNotActive(CoordinatorReleaseStatus),
    EmptyQualificationWindow,
}

impl fmt::Display for CoordinatorReleaseCurrentnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator-release currentness protocol"),
            Self::WrongManifestProfile => write!(f, "wrong coordinator-release manifest profile"),
            Self::WrongRegistryHeadProfile => write!(f, "wrong release-registry head profile"),
            Self::WrongStatusRecordProfile => write!(f, "wrong release-status record profile"),
            Self::InvalidDigest(field) => write!(f, "invalid {field}"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::InvalidRegistryGeneration => {
                write!(f, "release registry generation must be non-zero")
            }
            Self::InvalidEvidenceWindow => write!(
                f,
                "release-currentness evidence is stale, future-dated or inverted"
            ),
            Self::AuthenticatedReleaseNotLive => {
                write!(f, "authenticated release lease is not live")
            }
            Self::ReleasePolicyMismatch => {
                write!(f, "release currentness evidence names another release policy")
            }
            Self::ManifestMismatch => write!(f, "release status proof names another manifest"),
            Self::StatusForAnotherRegistryHead => write!(
                f,
                "release status proof is not for the independently verified current registry head"
            ),
            Self::ReleaseNotActive(status) => write!(f, "release is not active: {status:?}"),
            Self::EmptyQualificationWindow => write!(
                f,
                "release-currentness qualification has no live evidence window"
            ),
        }
    }
}

impl std::error::Error for CoordinatorReleaseCurrentnessError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_coordinator_release::{
        qualify_coordinator_release, CoordinatorReleaseManifest,
        VerifiedCoordinatorReleaseSignatureProof, MANIFEST_PROFILE as RELEASE_MANIFEST_PROFILE,
        PROTOCOL_VERSION as RELEASE_PROTOCOL_VERSION, SIGNATURE_PROOF_PROTOCOL,
    };
    use mycelix_authority_coordinator_deployment::CoordinatorCodeIdentity;

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn release() -> QualifiedCoordinatorReleaseRequirement {
        let manifest = CoordinatorReleaseManifest {
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
            release_authority_ref: "release-authority:1".into(),
            release_policy_digest: d(10),
            release_policy_profile: "release-policy-v1".into(),
            valid_from_ms: 100,
            valid_until_ms: 1_000,
        };
        let proof = VerifiedCoordinatorReleaseSignatureProof {
            protocol_version: SIGNATURE_PROOF_PROTOCOL.into(),
            manifest_digest: manifest.manifest_digest().unwrap(),
            manifest_profile: RELEASE_MANIFEST_PROFILE.into(),
            release_authority_ref: manifest.release_authority_ref.clone(),
            release_policy_digest: manifest.release_policy_digest,
            release_policy_profile: manifest.release_policy_profile.clone(),
            signing_key_id: "key:1".into(),
            signature_ref: "signature:1".into(),
            verifier_ref: "signature-verifier:1".into(),
            verified_at_ms: 120,
            valid_until_ms: 800,
        };
        qualify_coordinator_release(manifest, &proof, 200).unwrap()
    }

    fn head() -> VerifiedCurrentReleaseRegistryHeadProof {
        VerifiedCurrentReleaseRegistryHeadProof {
            protocol_version: PROTOCOL_VERSION.into(),
            release_policy_digest: d(10),
            release_policy_profile: "release-policy-v1".into(),
            registry_generation: 7,
            registry_head_digest: d(11),
            registry_head_profile: REGISTRY_HEAD_PROFILE.into(),
            source_ref: "release-registry:head:7".into(),
            verifier_ref: "release-registry-head-verifier:1".into(),
            verified_at_ms: 150,
            valid_until_ms: 500,
        }
    }

    fn status(
        release: &QualifiedCoordinatorReleaseRequirement,
    ) -> VerifiedCoordinatorReleaseStatusAtHeadProof {
        let head = head();
        VerifiedCoordinatorReleaseStatusAtHeadProof {
            protocol_version: PROTOCOL_VERSION.into(),
            manifest_digest: release.manifest_digest(),
            manifest_profile: MANIFEST_PROFILE.into(),
            release_policy_digest: head.release_policy_digest,
            release_policy_profile: head.release_policy_profile,
            registry_generation: head.registry_generation,
            registry_head_digest: head.registry_head_digest,
            registry_head_profile: head.registry_head_profile,
            status: CoordinatorReleaseStatus::Active,
            status_record_digest: d(12),
            status_record_profile: STATUS_RECORD_PROFILE.into(),
            verifier_ref: "release-status-verifier:1".into(),
            verified_at_ms: 160,
            valid_until_ms: 450,
        }
    }

    #[test]
    fn active_status_at_exact_current_head_qualifies() {
        let release = release();
        let head = head();
        let status = status(&release);
        let qualified = qualify_current_coordinator_release(release, &head, &status, 200).unwrap();
        assert_eq!(qualified.registry_generation(), 7);
        assert_eq!(qualified.valid_until_ms(), 450);
        assert_ne!(qualified.qualification_digest(), [0; 32]);
    }

    #[test]
    fn withdrawn_or_superseded_release_denies() {
        for state in [
            CoordinatorReleaseStatus::Withdrawn,
            CoordinatorReleaseStatus::Superseded,
        ] {
            let release = release();
            let head = head();
            let mut status = status(&release);
            status.status = state;
            assert_eq!(
                qualify_current_coordinator_release(release, &head, &status, 200).unwrap_err(),
                CoordinatorReleaseCurrentnessError::ReleaseNotActive(state)
            );
        }
    }

    #[test]
    fn active_proof_for_old_head_denies() {
        let release = release();
        let head = head();
        let mut status = status(&release);
        status.registry_generation = 6;
        status.registry_head_digest = d(99);
        assert_eq!(
            qualify_current_coordinator_release(release, &head, &status, 200).unwrap_err(),
            CoordinatorReleaseCurrentnessError::StatusForAnotherRegistryHead
        );
    }

    #[test]
    fn fixed_wire_profiles_are_mandatory() {
        let mut wrong_head = head();
        wrong_head.registry_head_profile = "alternate-head-v1".into();
        assert_eq!(
            wrong_head.validate_at(200).unwrap_err(),
            CoordinatorReleaseCurrentnessError::WrongRegistryHeadProfile
        );

        let release = release();
        let mut wrong_status = status(&release);
        wrong_status.status_record_profile = "alternate-status-v1".into();
        assert_eq!(
            wrong_status.validate_at(200).unwrap_err(),
            CoordinatorReleaseCurrentnessError::WrongStatusRecordProfile
        );

        let mut wrong_manifest = status(&release);
        wrong_manifest.manifest_profile = "alternate-manifest-v1".into();
        assert_eq!(
            wrong_manifest.validate_at(200).unwrap_err(),
            CoordinatorReleaseCurrentnessError::WrongManifestProfile
        );
    }

    #[test]
    fn policy_mismatch_denies() {
        let release = release();
        let mut head = head();
        head.release_policy_digest = d(77);
        let status = status(&release);
        assert_eq!(
            qualify_current_coordinator_release(release, &head, &status, 200).unwrap_err(),
            CoordinatorReleaseCurrentnessError::ReleasePolicyMismatch
        );
    }

    #[test]
    fn shortest_evidence_lease_caps_current_release() {
        let release = release();
        let mut head = head();
        head.valid_until_ms = 230;
        let mut status = status(&release);
        status.valid_until_ms = 240;
        let qualified = qualify_current_coordinator_release(release, &head, &status, 200).unwrap();
        assert_eq!(qualified.valid_until_ms(), 230);
    }
}

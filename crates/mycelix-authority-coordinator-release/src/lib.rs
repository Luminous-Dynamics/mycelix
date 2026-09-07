// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure authentication contract for an approved Mycelix coordinator release.
//!
//! This crate separates release semantics from signature verification and from
//! release currentness/withdrawal. A successful qualification proves only that an
//! independent verifier authenticated one exact release manifest and that its
//! bounded proof is live at `now_ms`. It does not prove that the release has not
//! subsequently been withdrawn or superseded.

use mycelix_authority_coordinator_deployment::{
    CoordinatorCodeIdentity, RequiredCoordinatorDeployment,
    PROTOCOL_VERSION as DEPLOYMENT_PROTOCOL_VERSION,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-release-v0.1";
pub const MANIFEST_PROFILE: &str =
    "mycelix-authority-coordinator-release-manifest-v1-blake3-framed";
pub const SIGNATURE_PROOF_PROTOCOL: &str =
    "mycelix-authority-coordinator-release-signature-proof-v0.1";
pub const SIGNATURE_PROOF_PROFILE: &str =
    "mycelix-authority-coordinator-release-signature-proof-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-release-qualification-v1-blake3-framed";

const DOMAIN_MANIFEST: &[u8] = b"mycelix/authority/coordinator-release-manifest/v1";
const DOMAIN_SIGNATURE_PROOF: &[u8] =
    b"mycelix/authority/coordinator-release-signature-proof/v1";
const DOMAIN_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-release-qualification/v1";
const HOLO_HASH_RAW_LEN: usize = 39;
const MAX_TEXT_BYTES: usize = 2048;
const MAX_ZOME_NAME_BYTES: usize = 128;
const MAX_COORDINATORS: usize = 128;

/// Signed semantic release statement. This is DNA-scoped approved code, not an
/// installation-specific CellId requirement: the target agent/cell is selected
/// independently when this release is specialized for deployment.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseManifest {
    pub protocol_version: String,
    pub release_id: String,
    pub release_version: u64,
    pub dna_hash_raw_39: Vec<u8>,
    pub coordinators: Vec<CoordinatorCodeIdentity>,
    pub dna_bundle_digest: [u8; 32],
    pub source_tree_digest: [u8; 32],
    pub lockfile_digest: [u8; 32],
    pub toolchain_digest: [u8; 32],
    pub build_recipe_digest: [u8; 32],
    pub sbom_digest: [u8; 32],
    pub source_ref: String,
    pub build_ref: String,
    pub release_authority_ref: String,
    pub release_policy_digest: [u8; 32],
    pub release_policy_profile: String,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl CoordinatorReleaseManifest {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorReleaseError::WrongProtocol);
        }
        validate_text(&self.release_id, "release id")?;
        validate_text(&self.source_ref, "source ref")?;
        validate_text(&self.build_ref, "build ref")?;
        validate_text(&self.release_authority_ref, "release authority ref")?;
        validate_text(&self.release_policy_profile, "release policy profile")?;
        if self.release_version == 0 {
            return Err(CoordinatorReleaseError::InvalidReleaseVersion);
        }
        validate_holo_hash(&self.dna_hash_raw_39, "DNA hash")?;
        validate_coordinators(&self.coordinators)?;
        for (name, digest) in [
            ("DNA bundle digest", &self.dna_bundle_digest),
            ("source tree digest", &self.source_tree_digest),
            ("lockfile digest", &self.lockfile_digest),
            ("toolchain digest", &self.toolchain_digest),
            ("build recipe digest", &self.build_recipe_digest),
            ("SBOM digest", &self.sbom_digest),
            ("release policy digest", &self.release_policy_digest),
        ] {
            validate_digest(digest, name)?;
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(CoordinatorReleaseError::InvalidManifestWindow);
        }
        Ok(())
    }

    pub fn manifest_digest(&self) -> Result<[u8; 32], CoordinatorReleaseError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_MANIFEST);
        frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame(&mut hasher, MANIFEST_PROFILE.as_bytes());
        frame(&mut hasher, self.release_id.as_bytes());
        frame(&mut hasher, &self.release_version.to_le_bytes());
        frame(&mut hasher, &self.dna_hash_raw_39);
        for coordinator in canonical_coordinators(&self.coordinators) {
            frame(&mut hasher, coordinator.zome_name.as_bytes());
            frame(&mut hasher, &coordinator.wasm_hash_raw_39);
        }
        for digest in [
            &self.dna_bundle_digest,
            &self.source_tree_digest,
            &self.lockfile_digest,
            &self.toolchain_digest,
            &self.build_recipe_digest,
            &self.sbom_digest,
        ] {
            frame(&mut hasher, digest);
        }
        frame(&mut hasher, self.source_ref.as_bytes());
        frame(&mut hasher, self.build_ref.as_bytes());
        frame(&mut hasher, self.release_authority_ref.as_bytes());
        frame(&mut hasher, &self.release_policy_digest);
        frame(&mut hasher, self.release_policy_profile.as_bytes());
        frame(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Evidence-shaped receipt emitted by an independent release-signature verifier.
///
/// This type is intentionally deserializable because it crosses the verifier
/// boundary. Its fields grant no authority until locally cross-bound to the exact
/// manifest by [`qualify_coordinator_release`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCoordinatorReleaseSignatureProof {
    pub protocol_version: String,
    pub manifest_digest: [u8; 32],
    pub manifest_profile: String,
    pub release_authority_ref: String,
    pub release_policy_digest: [u8; 32],
    pub release_policy_profile: String,
    pub signing_key_id: String,
    pub signature_ref: String,
    pub verifier_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedCoordinatorReleaseSignatureProof {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), CoordinatorReleaseError> {
        if self.protocol_version != SIGNATURE_PROOF_PROTOCOL {
            return Err(CoordinatorReleaseError::WrongSignatureProofProtocol);
        }
        if self.manifest_profile != MANIFEST_PROFILE {
            return Err(CoordinatorReleaseError::WrongManifestProfile);
        }
        validate_digest(&self.manifest_digest, "manifest digest")?;
        validate_digest(&self.release_policy_digest, "release policy digest")?;
        for (value, field) in [
            (&self.release_authority_ref, "release authority ref"),
            (&self.release_policy_profile, "release policy profile"),
            (&self.signing_key_id, "signing key id"),
            (&self.signature_ref, "signature ref"),
            (&self.verifier_ref, "signature verifier ref"),
        ] {
            validate_text(value, field)?;
        }
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms <= self.verified_at_ms
        {
            return Err(CoordinatorReleaseError::InvalidSignatureProofWindow);
        }
        Ok(())
    }

    pub fn proof_digest(&self) -> Result<[u8; 32], CoordinatorReleaseError> {
        // `valid_until_ms - 1` is a valid observation instant when the window is
        // nonempty and avoids inventing a second current clock for shape hashing.
        let shape_now = self
            .valid_until_ms
            .checked_sub(1)
            .ok_or(CoordinatorReleaseError::InvalidSignatureProofWindow)?;
        self.validate_at(shape_now)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_SIGNATURE_PROOF);
        frame(&mut hasher, SIGNATURE_PROOF_PROTOCOL.as_bytes());
        frame(&mut hasher, SIGNATURE_PROOF_PROFILE.as_bytes());
        frame(&mut hasher, &self.manifest_digest);
        frame(&mut hasher, self.manifest_profile.as_bytes());
        frame(&mut hasher, self.release_authority_ref.as_bytes());
        frame(&mut hasher, &self.release_policy_digest);
        frame(&mut hasher, self.release_policy_profile.as_bytes());
        frame(&mut hasher, self.signing_key_id.as_bytes());
        frame(&mut hasher, self.signature_ref.as_bytes());
        frame(&mut hasher, self.verifier_ref.as_bytes());
        frame(&mut hasher, &self.verified_at_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Non-deserializable authentication of one exact coordinator release manifest.
///
/// This proves signature authentication plus lease containment. It does not prove
/// release currentness/non-withdrawal and therefore is not by itself sufficient
/// for live deployment/effect admission.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoordinatorReleaseRequirement {
    manifest: CoordinatorReleaseManifest,
    manifest_digest: [u8; 32],
    manifest_profile: String,
    signature_proof_digest: [u8; 32],
    signature_proof_profile: String,
    qualification_digest: [u8; 32],
    qualification_profile: String,
    verifier_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoordinatorReleaseRequirement {
    pub fn manifest(&self) -> &CoordinatorReleaseManifest {
        &self.manifest
    }

    pub fn manifest_digest(&self) -> [u8; 32] {
        self.manifest_digest
    }

    pub fn manifest_profile(&self) -> &str {
        &self.manifest_profile
    }

    pub fn signature_proof_digest(&self) -> [u8; 32] {
        self.signature_proof_digest
    }

    pub fn signature_proof_profile(&self) -> &str {
        &self.signature_proof_profile
    }

    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn verifier_ref(&self) -> &str {
        &self.verifier_ref
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    /// Specialize this DNA-scoped authenticated code release to one independently
    /// selected target cell agent. Target-cell provenance is intentionally outside
    /// the release signature theorem.
    pub fn required_deployment_for_target_agent(
        &self,
        agent_pub_key_raw_39: Vec<u8>,
    ) -> Result<RequiredCoordinatorDeployment, CoordinatorReleaseError> {
        let required = RequiredCoordinatorDeployment {
            protocol_version: DEPLOYMENT_PROTOCOL_VERSION.into(),
            dna_hash_raw_39: self.manifest.dna_hash_raw_39.clone(),
            agent_pub_key_raw_39,
            coordinators: self.manifest.coordinators.clone(),
        };
        required
            .validate()
            .map_err(|error| CoordinatorReleaseError::DeploymentRequirement(error.to_string()))?;
        Ok(required)
    }
}

pub fn qualify_coordinator_release(
    manifest: CoordinatorReleaseManifest,
    signature_proof: &VerifiedCoordinatorReleaseSignatureProof,
    now_ms: u64,
) -> Result<QualifiedCoordinatorReleaseRequirement, CoordinatorReleaseError> {
    manifest.validate()?;
    signature_proof.validate_at(now_ms)?;
    if now_ms < manifest.valid_from_ms || now_ms >= manifest.valid_until_ms {
        return Err(CoordinatorReleaseError::ManifestNotLive);
    }

    let manifest_digest = manifest.manifest_digest()?;
    if signature_proof.manifest_digest != manifest_digest {
        return Err(CoordinatorReleaseError::ManifestDigestMismatch);
    }
    if signature_proof.manifest_profile != MANIFEST_PROFILE {
        return Err(CoordinatorReleaseError::WrongManifestProfile);
    }
    if signature_proof.release_authority_ref != manifest.release_authority_ref {
        return Err(CoordinatorReleaseError::ReleaseAuthorityMismatch);
    }
    if signature_proof.release_policy_digest != manifest.release_policy_digest
        || signature_proof.release_policy_profile != manifest.release_policy_profile
    {
        return Err(CoordinatorReleaseError::ReleasePolicyMismatch);
    }

    let verified_at_ms = signature_proof.verified_at_ms.max(manifest.valid_from_ms);
    let valid_until_ms = signature_proof.valid_until_ms.min(manifest.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms || valid_until_ms <= verified_at_ms {
        return Err(CoordinatorReleaseError::EmptyQualificationWindow);
    }

    let signature_proof_digest = signature_proof.proof_digest()?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &manifest_digest);
    frame(&mut hasher, MANIFEST_PROFILE.as_bytes());
    frame(&mut hasher, &signature_proof_digest);
    frame(&mut hasher, SIGNATURE_PROOF_PROFILE.as_bytes());
    frame(&mut hasher, signature_proof.verifier_ref.as_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    let qualification_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCoordinatorReleaseRequirement {
        manifest,
        manifest_digest,
        manifest_profile: MANIFEST_PROFILE.into(),
        signature_proof_digest,
        signature_proof_profile: SIGNATURE_PROOF_PROFILE.into(),
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        verifier_ref: signature_proof.verifier_ref.clone(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn validate_holo_hash(value: &[u8], field: &'static str) -> Result<(), CoordinatorReleaseError> {
    if value.len() != HOLO_HASH_RAW_LEN || value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorReleaseError::InvalidHash(field))
    } else {
        Ok(())
    }
}

fn validate_digest(value: &[u8; 32], field: &'static str) -> Result<(), CoordinatorReleaseError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorReleaseError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), CoordinatorReleaseError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(CoordinatorReleaseError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn validate_coordinators(values: &[CoordinatorCodeIdentity]) -> Result<(), CoordinatorReleaseError> {
    if values.is_empty() || values.len() > MAX_COORDINATORS {
        return Err(CoordinatorReleaseError::InvalidCoordinatorSet);
    }
    let canonical = canonical_coordinators(values);
    for value in &canonical {
        if value.zome_name.is_empty()
            || value.zome_name.len() > MAX_ZOME_NAME_BYTES
            || value.zome_name.bytes().any(|byte| byte.is_ascii_whitespace())
        {
            return Err(CoordinatorReleaseError::InvalidCoordinatorName);
        }
        validate_holo_hash(&value.wasm_hash_raw_39, "coordinator WASM hash")?;
    }
    for pair in canonical.windows(2) {
        if pair[0].zome_name == pair[1].zome_name {
            return Err(CoordinatorReleaseError::DuplicateCoordinator(
                pair[0].zome_name.clone(),
            ));
        }
    }
    Ok(())
}

fn canonical_coordinators(values: &[CoordinatorCodeIdentity]) -> Vec<CoordinatorCodeIdentity> {
    let mut canonical = values.to_vec();
    canonical.sort_by(|left, right| left.zome_name.cmp(&right.zome_name));
    canonical
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorReleaseError {
    WrongProtocol,
    WrongSignatureProofProtocol,
    WrongManifestProfile,
    InvalidReleaseVersion,
    InvalidHash(&'static str),
    InvalidDigest(&'static str),
    InvalidText(&'static str),
    InvalidCoordinatorName,
    InvalidCoordinatorSet,
    DuplicateCoordinator(String),
    InvalidManifestWindow,
    InvalidSignatureProofWindow,
    ManifestNotLive,
    ManifestDigestMismatch,
    ReleaseAuthorityMismatch,
    ReleasePolicyMismatch,
    EmptyQualificationWindow,
    DeploymentRequirement(String),
}

impl fmt::Display for CoordinatorReleaseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator-release protocol"),
            Self::WrongSignatureProofProtocol => write!(f, "wrong release-signature proof protocol"),
            Self::WrongManifestProfile => write!(f, "wrong release manifest profile"),
            Self::InvalidReleaseVersion => write!(f, "release version must be non-zero"),
            Self::InvalidHash(field) => write!(f, "invalid {field}"),
            Self::InvalidDigest(field) => write!(f, "invalid {field}"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::InvalidCoordinatorName => write!(f, "invalid coordinator zome name"),
            Self::InvalidCoordinatorSet => write!(f, "invalid approved coordinator set"),
            Self::DuplicateCoordinator(name) => write!(f, "duplicate coordinator {name}"),
            Self::InvalidManifestWindow => write!(f, "invalid release manifest validity window"),
            Self::InvalidSignatureProofWindow => write!(f, "invalid release-signature proof window"),
            Self::ManifestNotLive => write!(f, "release manifest is not live at qualification time"),
            Self::ManifestDigestMismatch => write!(f, "signature proof names another release manifest"),
            Self::ReleaseAuthorityMismatch => write!(f, "signature proof names another release authority"),
            Self::ReleasePolicyMismatch => write!(f, "signature proof names another release policy"),
            Self::EmptyQualificationWindow => write!(f, "release qualification has no live evidence window"),
            Self::DeploymentRequirement(error) => write!(f, "target deployment requirement denied: {error}"),
        }
    }
}

impl std::error::Error for CoordinatorReleaseError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; HOLO_HASH_RAW_LEN]
    }

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn c(name: &str, byte: u8) -> CoordinatorCodeIdentity {
        CoordinatorCodeIdentity {
            zome_name: name.into(),
            wasm_hash_raw_39: h(byte),
        }
    }

    fn manifest() -> CoordinatorReleaseManifest {
        CoordinatorReleaseManifest {
            protocol_version: PROTOCOL_VERSION.into(),
            release_id: "governance-authority-v1".into(),
            release_version: 1,
            dna_hash_raw_39: h(9),
            coordinators: vec![c("constitution_currentness_verifier", 2), c("authority_current_freshness_verifier", 1)],
            dna_bundle_digest: d(3),
            source_tree_digest: d(4),
            lockfile_digest: d(5),
            toolchain_digest: d(6),
            build_recipe_digest: d(7),
            sbom_digest: d(8),
            source_ref: "git:example:tree:abc".into(),
            build_ref: "nix:governance-release:abc".into(),
            release_authority_ref: "release-authority:governance".into(),
            release_policy_digest: d(10),
            release_policy_profile: "mycelix-release-policy-v1".into(),
            valid_from_ms: 100,
            valid_until_ms: 1_000,
        }
    }

    fn proof(manifest: &CoordinatorReleaseManifest) -> VerifiedCoordinatorReleaseSignatureProof {
        VerifiedCoordinatorReleaseSignatureProof {
            protocol_version: SIGNATURE_PROOF_PROTOCOL.into(),
            manifest_digest: manifest.manifest_digest().unwrap(),
            manifest_profile: MANIFEST_PROFILE.into(),
            release_authority_ref: manifest.release_authority_ref.clone(),
            release_policy_digest: manifest.release_policy_digest,
            release_policy_profile: manifest.release_policy_profile.clone(),
            signing_key_id: "key:release:1".into(),
            signature_ref: "signature:release:1".into(),
            verifier_ref: "release-signature-verifier:1".into(),
            verified_at_ms: 120,
            valid_until_ms: 400,
        }
    }

    #[test]
    fn exact_signature_proof_qualifies_and_caps_lease() {
        let manifest = manifest();
        let proof = proof(&manifest);
        let qualified = qualify_coordinator_release(manifest.clone(), &proof, 200).unwrap();
        assert_eq!(qualified.manifest_digest(), manifest.manifest_digest().unwrap());
        assert_eq!(qualified.verified_at_ms(), 120);
        assert_eq!(qualified.valid_until_ms(), 400);
        assert_eq!(qualified.manifest_profile(), MANIFEST_PROFILE);
        assert_eq!(qualified.signature_proof_profile(), SIGNATURE_PROOF_PROFILE);
    }

    #[test]
    fn manifest_identity_is_coordinator_order_independent() {
        let first = manifest();
        let mut second = first.clone();
        second.coordinators.reverse();
        assert_eq!(first.manifest_digest().unwrap(), second.manifest_digest().unwrap());
    }

    #[test]
    fn changed_manifest_after_signature_denies() {
        let manifest = manifest();
        let proof = proof(&manifest);
        let mut changed = manifest.clone();
        changed.sbom_digest = d(99);
        assert_eq!(
            qualify_coordinator_release(changed, &proof, 200).unwrap_err(),
            CoordinatorReleaseError::ManifestDigestMismatch
        );
    }

    #[test]
    fn policy_or_authority_substitution_denies() {
        let manifest = manifest();
        let mut wrong_authority = proof(&manifest);
        wrong_authority.release_authority_ref = "release-authority:other".into();
        assert_eq!(
            qualify_coordinator_release(manifest.clone(), &wrong_authority, 200).unwrap_err(),
            CoordinatorReleaseError::ReleaseAuthorityMismatch
        );

        let mut wrong_policy = proof(&manifest);
        wrong_policy.release_policy_digest = d(77);
        assert_eq!(
            qualify_coordinator_release(manifest.clone(), &wrong_policy, 200).unwrap_err(),
            CoordinatorReleaseError::ReleasePolicyMismatch
        );
    }

    #[test]
    fn short_signature_proof_is_valid_now_without_widening() {
        let manifest = manifest();
        let mut proof = proof(&manifest);
        proof.valid_until_ms = 205;
        let qualified = qualify_coordinator_release(manifest, &proof, 200).unwrap();
        assert_eq!(qualified.valid_until_ms(), 205);
    }

    #[test]
    fn exact_release_specializes_to_independently_selected_cell_agent() {
        let manifest = manifest();
        let proof = proof(&manifest);
        let qualified = qualify_coordinator_release(manifest.clone(), &proof, 200).unwrap();
        let required = qualified
            .required_deployment_for_target_agent(h(42))
            .unwrap();
        assert_eq!(required.dna_hash_raw_39, manifest.dna_hash_raw_39);
        assert_eq!(required.agent_pub_key_raw_39, h(42));
        assert_eq!(required.coordinators, manifest.coordinators);
    }

    #[test]
    fn duplicate_or_unusable_release_material_denies() {
        let mut duplicate = manifest();
        duplicate.coordinators.push(c("constitution_currentness_verifier", 2));
        assert!(matches!(
            duplicate.validate(),
            Err(CoordinatorReleaseError::DuplicateCoordinator(_))
        ));

        let mut zero_digest = manifest();
        zero_digest.lockfile_digest = [0; 32];
        assert!(matches!(
            zero_digest.validate(),
            Err(CoordinatorReleaseError::InvalidDigest("lockfile digest"))
        ));
    }

    #[test]
    fn serde_signature_receipt_does_not_create_qualified_release() {
        let manifest = manifest();
        let proof = proof(&manifest);
        let json = serde_json::to_string(&proof).unwrap();
        let decoded: VerifiedCoordinatorReleaseSignatureProof = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, proof);
        let qualified = qualify_coordinator_release(manifest, &decoded, 200).unwrap();
        assert_ne!(qualified.qualification_digest(), [0; 32]);
    }
}

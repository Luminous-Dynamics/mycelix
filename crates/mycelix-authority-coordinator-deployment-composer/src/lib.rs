// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure exact composition of current approved coordinator code with one observed
//! Holochain cell deployment.
//!
//! This crate proves equality and lease containment only. It does not prove the
//! provenance of the target-cell selection, the native conductor observation, or
//! the verifier receipts that established release authenticity/currentness.

use mycelix_authority_coordinator_deployment::{
    match_required_coordinator_deployment, MatchedCoordinatorDeployment,
    ObservedCoordinatorDeployment, REQUIREMENT_PROFILE,
};
use mycelix_authority_coordinator_release_currentness::QualifiedCurrentCoordinatorRelease;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-deployment-composer-v0.1";
pub const TARGET_SELECTION_PROFILE: &str =
    "mycelix-authority-coordinator-target-cell-selection-v1-blake3-framed";
pub const COMPOSITION_PROFILE: &str =
    "mycelix-authority-coordinator-deployment-composition-v1-blake3-framed";

const DOMAIN_TARGET: &[u8] = b"mycelix/authority/coordinator-target-cell/v1";
const DOMAIN_COMPOSITION: &[u8] = b"mycelix/authority/coordinator-deployment-composition/v1";
const HOLO_HASH_RAW_LEN: usize = 39;
const MAX_REF_BYTES: usize = 2048;

/// Independently supplied exact target CellId.
///
/// This is intentionally deserializable evidence/data. Its presence proves no
/// target-selection provenance; the eventual native admission path must establish
/// why this exact CellId is the cell being admitted.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TargetCellSelection {
    pub protocol_version: String,
    pub dna_hash_raw_39: Vec<u8>,
    pub agent_pub_key_raw_39: Vec<u8>,
    pub selection_ref: String,
}

impl TargetCellSelection {
    pub fn validate(&self) -> Result<(), CoordinatorDeploymentCompositionError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorDeploymentCompositionError::WrongProtocol);
        }
        validate_holo_hash(&self.dna_hash_raw_39, "target DNA hash")?;
        validate_holo_hash(&self.agent_pub_key_raw_39, "target cell agent key")?;
        validate_ref(&self.selection_ref)?;
        Ok(())
    }

    pub fn selection_digest(&self) -> Result<[u8; 32], CoordinatorDeploymentCompositionError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_TARGET);
        frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame(&mut hasher, TARGET_SELECTION_PROFILE.as_bytes());
        frame(&mut hasher, &self.dna_hash_raw_39);
        frame(&mut hasher, &self.agent_pub_key_raw_39);
        frame(&mut hasher, self.selection_ref.as_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Non-deserializable equality proof over:
///
/// - one already-qualified current release;
/// - one independently supplied target CellId; and
/// - one exact observed conductor deployment.
///
/// It remains deployment evidence, not execution/effect authority and not proof of
/// the external provenance of any of those three input domains.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoordinatorDeploymentComposition {
    current_release: QualifiedCurrentCoordinatorRelease,
    matched_deployment: MatchedCoordinatorDeployment,
    target_selection_digest: [u8; 32],
    target_selection_profile: String,
    target_dna_hash_raw_39: Vec<u8>,
    target_agent_pub_key_raw_39: Vec<u8>,
    required_deployment_digest: [u8; 32],
    required_deployment_profile: String,
    composition_digest: [u8; 32],
    composition_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoordinatorDeploymentComposition {
    pub fn current_release(&self) -> &QualifiedCurrentCoordinatorRelease {
        &self.current_release
    }

    pub fn matched_deployment(&self) -> &MatchedCoordinatorDeployment {
        &self.matched_deployment
    }

    pub fn target_selection_digest(&self) -> [u8; 32] {
        self.target_selection_digest
    }

    pub fn target_selection_profile(&self) -> &str {
        &self.target_selection_profile
    }

    pub fn target_dna_hash_raw_39(&self) -> &[u8] {
        &self.target_dna_hash_raw_39
    }

    pub fn target_agent_pub_key_raw_39(&self) -> &[u8] {
        &self.target_agent_pub_key_raw_39
    }

    pub fn required_deployment_digest(&self) -> [u8; 32] {
        self.required_deployment_digest
    }

    pub fn required_deployment_profile(&self) -> &str {
        &self.required_deployment_profile
    }

    pub fn composition_digest(&self) -> [u8; 32] {
        self.composition_digest
    }

    pub fn composition_profile(&self) -> &str {
        &self.composition_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }
}

pub fn qualify_current_release_against_observed_deployment(
    current_release: QualifiedCurrentCoordinatorRelease,
    target: &TargetCellSelection,
    observed: &ObservedCoordinatorDeployment,
    now_ms: u64,
) -> Result<QualifiedCoordinatorDeploymentComposition, CoordinatorDeploymentCompositionError> {
    target.validate()?;
    if current_release.verified_at_ms() > now_ms || current_release.valid_until_ms() <= now_ms {
        return Err(CoordinatorDeploymentCompositionError::CurrentReleaseNotLive);
    }

    let manifest = current_release.release().manifest();
    if target.dna_hash_raw_39 != manifest.dna_hash_raw_39 {
        return Err(CoordinatorDeploymentCompositionError::TargetDnaMismatch);
    }

    let required = current_release
        .release()
        .required_deployment_for_target_agent(target.agent_pub_key_raw_39.clone())
        .map_err(|error| {
            CoordinatorDeploymentCompositionError::ReleaseSpecialization(error.to_string())
        })?;
    if required.dna_hash_raw_39 != target.dna_hash_raw_39
        || required.agent_pub_key_raw_39 != target.agent_pub_key_raw_39
    {
        return Err(CoordinatorDeploymentCompositionError::TargetSpecializationMismatch);
    }

    let matched = match_required_coordinator_deployment(&required, observed, now_ms)
        .map_err(CoordinatorDeploymentCompositionError::DeploymentMatch)?;
    if matched.dna_hash_raw_39() != target.dna_hash_raw_39.as_slice()
        || matched.agent_pub_key_raw_39() != target.agent_pub_key_raw_39.as_slice()
    {
        return Err(CoordinatorDeploymentCompositionError::MatchedCellMismatch);
    }

    let verified_at_ms = current_release
        .verified_at_ms()
        .max(matched.observed_at_ms());
    let valid_until_ms = current_release
        .valid_until_ms()
        .min(matched.valid_until_ms());
    if verified_at_ms > now_ms || valid_until_ms <= now_ms || valid_until_ms <= verified_at_ms {
        return Err(CoordinatorDeploymentCompositionError::EmptyCompositionWindow);
    }

    let target_selection_digest = target.selection_digest()?;
    let required_deployment_digest = required
        .requirement_digest()
        .map_err(CoordinatorDeploymentCompositionError::DeploymentMatch)?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_COMPOSITION);
    frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, COMPOSITION_PROFILE.as_bytes());
    frame(&mut hasher, &current_release.qualification_digest());
    frame(
        &mut hasher,
        current_release.qualification_profile().as_bytes(),
    );
    frame(&mut hasher, &target_selection_digest);
    frame(&mut hasher, TARGET_SELECTION_PROFILE.as_bytes());
    frame(&mut hasher, &required_deployment_digest);
    frame(&mut hasher, REQUIREMENT_PROFILE.as_bytes());
    frame(&mut hasher, &matched.match_digest());
    frame(&mut hasher, matched.match_profile().as_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    let composition_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCoordinatorDeploymentComposition {
        current_release,
        target_selection_digest,
        target_selection_profile: TARGET_SELECTION_PROFILE.into(),
        target_dna_hash_raw_39: target.dna_hash_raw_39.clone(),
        target_agent_pub_key_raw_39: target.agent_pub_key_raw_39.clone(),
        required_deployment_digest,
        required_deployment_profile: REQUIREMENT_PROFILE.into(),
        matched_deployment: matched,
        composition_digest,
        composition_profile: COMPOSITION_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn validate_holo_hash(
    value: &[u8],
    field: &'static str,
) -> Result<(), CoordinatorDeploymentCompositionError> {
    if value.len() != HOLO_HASH_RAW_LEN || value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorDeploymentCompositionError::InvalidHash(field))
    } else {
        Ok(())
    }
}

fn validate_ref(value: &str) -> Result<(), CoordinatorDeploymentCompositionError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(CoordinatorDeploymentCompositionError::InvalidSelectionRef)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorDeploymentCompositionError {
    WrongProtocol,
    InvalidHash(&'static str),
    InvalidSelectionRef,
    CurrentReleaseNotLive,
    TargetDnaMismatch,
    ReleaseSpecialization(String),
    TargetSpecializationMismatch,
    DeploymentMatch(mycelix_authority_coordinator_deployment::CoordinatorDeploymentError),
    MatchedCellMismatch,
    EmptyCompositionWindow,
}

impl fmt::Display for CoordinatorDeploymentCompositionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator deployment composer protocol"),
            Self::InvalidHash(field) => write!(f, "invalid {field}"),
            Self::InvalidSelectionRef => write!(f, "invalid target-cell selection reference"),
            Self::CurrentReleaseNotLive => write!(f, "current coordinator release is not live"),
            Self::TargetDnaMismatch => write!(f, "target cell DNA does not match the current release"),
            Self::ReleaseSpecialization(error) => {
                write!(f, "current release could not specialize to the target cell: {error}")
            }
            Self::TargetSpecializationMismatch => {
                write!(f, "release specialization did not preserve the exact target CellId")
            }
            Self::DeploymentMatch(error) => write!(f, "coordinator deployment match denied: {error}"),
            Self::MatchedCellMismatch => write!(f, "matched deployment does not equal the exact target CellId"),
            Self::EmptyCompositionWindow => write!(f, "coordinator deployment composition has no live evidence window"),
        }
    }
}

impl std::error::Error for CoordinatorDeploymentCompositionError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_coordinator_deployment::{
        CoordinatorCodeIdentity, ObservedCoordinatorDeployment, CONDUCTOR_ADMIN_SOURCE_PROFILE,
        PROTOCOL_VERSION as DEPLOYMENT_PROTOCOL_VERSION,
    };
    use mycelix_authority_coordinator_release::{
        qualify_coordinator_release, CoordinatorReleaseManifest,
        VerifiedCoordinatorReleaseSignatureProof, MANIFEST_PROFILE,
        PROTOCOL_VERSION as RELEASE_PROTOCOL_VERSION, SIGNATURE_PROOF_PROTOCOL,
    };
    use mycelix_authority_coordinator_release_currentness::{
        qualify_current_coordinator_release, CoordinatorReleaseStatus,
        VerifiedCoordinatorReleaseStatusAtHeadProof, VerifiedCurrentReleaseRegistryHeadProof,
        PROTOCOL_VERSION as CURRENTNESS_PROTOCOL_VERSION, REGISTRY_HEAD_PROFILE,
        STATUS_RECORD_PROFILE,
    };

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn coordinator(name: &str, byte: u8) -> CoordinatorCodeIdentity {
        CoordinatorCodeIdentity {
            zome_name: name.into(),
            wasm_hash_raw_39: h(byte),
        }
    }

    fn current_release() -> QualifiedCurrentCoordinatorRelease {
        let manifest = CoordinatorReleaseManifest {
            protocol_version: RELEASE_PROTOCOL_VERSION.into(),
            release_id: "release-1".into(),
            release_version: 1,
            dna_hash_raw_39: h(9),
            coordinators: vec![coordinator("authority_current_freshness_verifier", 1)],
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
        let signature = VerifiedCoordinatorReleaseSignatureProof {
            protocol_version: SIGNATURE_PROOF_PROTOCOL.into(),
            manifest_digest: manifest.manifest_digest().unwrap(),
            manifest_profile: MANIFEST_PROFILE.into(),
            release_authority_ref: manifest.release_authority_ref.clone(),
            release_policy_digest: manifest.release_policy_digest,
            release_policy_profile: manifest.release_policy_profile.clone(),
            signing_key_id: "key:1".into(),
            signature_ref: "signature:1".into(),
            verifier_ref: "signature-verifier:1".into(),
            verified_at_ms: 120,
            valid_until_ms: 800,
        };
        let release = qualify_coordinator_release(manifest, &signature, 200).unwrap();
        let head = VerifiedCurrentReleaseRegistryHeadProof {
            protocol_version: CURRENTNESS_PROTOCOL_VERSION.into(),
            release_policy_digest: d(10),
            release_policy_profile: "release-policy-v1".into(),
            registry_generation: 7,
            registry_head_digest: d(11),
            registry_head_profile: REGISTRY_HEAD_PROFILE.into(),
            source_ref: "release-registry:head:7".into(),
            verifier_ref: "release-head-verifier:1".into(),
            verified_at_ms: 150,
            valid_until_ms: 500,
        };
        let status = VerifiedCoordinatorReleaseStatusAtHeadProof {
            protocol_version: CURRENTNESS_PROTOCOL_VERSION.into(),
            manifest_digest: release.manifest_digest(),
            manifest_profile: MANIFEST_PROFILE.into(),
            release_policy_digest: d(10),
            release_policy_profile: "release-policy-v1".into(),
            registry_generation: 7,
            registry_head_digest: d(11),
            registry_head_profile: REGISTRY_HEAD_PROFILE.into(),
            status: CoordinatorReleaseStatus::Active,
            status_record_digest: d(12),
            status_record_profile: STATUS_RECORD_PROFILE.into(),
            verifier_ref: "release-status-verifier:1".into(),
            verified_at_ms: 160,
            valid_until_ms: 450,
        };
        qualify_current_coordinator_release(release, &head, &status, 200).unwrap()
    }

    fn target() -> TargetCellSelection {
        TargetCellSelection {
            protocol_version: PROTOCOL_VERSION.into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            selection_ref: "target-cell:governance:primary".into(),
        }
    }

    fn observed() -> ObservedCoordinatorDeployment {
        ObservedCoordinatorDeployment {
            protocol_version: DEPLOYMENT_PROTOCOL_VERSION.into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            coordinators: vec![coordinator("authority_current_freshness_verifier", 1)],
            source_profile: CONDUCTOR_ADMIN_SOURCE_PROFILE.into(),
            source_ref: "admin-websocket:loopback:cell:test".into(),
            observed_at_ms: 190,
            valid_until_ms: 300,
        }
    }

    #[test]
    fn current_release_exact_target_and_observed_set_compose() {
        let composed = qualify_current_release_against_observed_deployment(
            current_release(),
            &target(),
            &observed(),
            200,
        )
        .unwrap();
        assert_eq!(composed.target_dna_hash_raw_39(), h(9));
        assert_eq!(composed.target_agent_pub_key_raw_39(), h(8));
        assert_eq!(composed.valid_until_ms(), 300);
        assert_ne!(composed.composition_digest(), [0; 32]);
    }

    #[test]
    fn target_selection_is_independent_and_must_match_release_and_observation() {
        let mut wrong_dna = target();
        wrong_dna.dna_hash_raw_39 = h(7);
        assert_eq!(
            qualify_current_release_against_observed_deployment(
                current_release(),
                &wrong_dna,
                &observed(),
                200,
            )
            .unwrap_err(),
            CoordinatorDeploymentCompositionError::TargetDnaMismatch
        );

        let mut wrong_observation = observed();
        wrong_observation.agent_pub_key_raw_39 = h(6);
        assert!(matches!(
            qualify_current_release_against_observed_deployment(
                current_release(),
                &target(),
                &wrong_observation,
                200,
            ),
            Err(CoordinatorDeploymentCompositionError::DeploymentMatch(_))
        ));
    }

    #[test]
    fn unexpected_or_substituted_coordinator_still_denies_through_exact_matcher() {
        let mut extra = observed();
        extra.coordinators.push(coordinator("unexpected", 4));
        assert!(matches!(
            qualify_current_release_against_observed_deployment(
                current_release(),
                &target(),
                &extra,
                200,
            ),
            Err(CoordinatorDeploymentCompositionError::DeploymentMatch(_))
        ));

        let mut substituted = observed();
        substituted.coordinators[0].wasm_hash_raw_39 = h(99);
        assert!(matches!(
            qualify_current_release_against_observed_deployment(
                current_release(),
                &target(),
                &substituted,
                200,
            ),
            Err(CoordinatorDeploymentCompositionError::DeploymentMatch(_))
        ));
    }

    #[test]
    fn shortest_current_release_or_observation_lease_wins() {
        let release = current_release();
        let mut observation = observed();
        observation.valid_until_ms = 205;
        let composed = qualify_current_release_against_observed_deployment(
            release,
            &target(),
            &observation,
            200,
        )
        .unwrap();
        assert_eq!(composed.valid_until_ms(), 205);
    }
}

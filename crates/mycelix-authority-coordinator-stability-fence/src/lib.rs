// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure before/after coordinator-deployment stability theorem for one admission attempt.
//!
//! This crate proves that one already-qualified exact deployment composition was
//! followed by a strictly later observation that still matches the same current
//! release and exact CellId. It does not prove that the observations came from the
//! native attestor or that they actually bracketed a real effect; those are native
//! orchestration responsibilities.

use mycelix_authority_coordinator_deployment::{
    match_required_coordinator_deployment, CoordinatorDeploymentError,
    MatchedCoordinatorDeployment, ObservedCoordinatorDeployment, REQUIREMENT_PROFILE,
};
use mycelix_authority_coordinator_deployment_composer::QualifiedCoordinatorDeploymentComposition;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-stability-fence-v0.1";
pub const ADMISSION_SUBJECT_PROFILE: &str =
    "mycelix-authority-coordinator-admission-subject-v1-blake3-framed";
pub const FENCE_PROFILE: &str =
    "mycelix-authority-coordinator-stability-fence-v1-blake3-framed";

const DOMAIN_SUBJECT: &[u8] = b"mycelix/authority/coordinator-admission-subject/v1";
const DOMAIN_FENCE: &[u8] = b"mycelix/authority/coordinator-stability-fence/v1";
const MAX_TEXT_BYTES: usize = 2048;

/// Exact admission subject and attempt identity to which a stability fence is bound.
///
/// This type is deliberately deserializable evidence/data. A live native admission
/// orchestrator must establish subject provenance and generate/own the attempt nonce.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorAdmissionSubject {
    pub protocol_version: String,
    pub subject_digest: [u8; 32],
    pub subject_profile: String,
    pub attempt_nonce: [u8; 32],
    pub attempt_ref: String,
}

impl CoordinatorAdmissionSubject {
    pub fn validate(&self) -> Result<(), CoordinatorStabilityFenceError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorStabilityFenceError::WrongProtocol);
        }
        validate_digest(&self.subject_digest, "admission subject digest")?;
        validate_text(&self.subject_profile, "admission subject profile")?;
        validate_digest(&self.attempt_nonce, "admission attempt nonce")?;
        validate_text(&self.attempt_ref, "admission attempt reference")?;
        Ok(())
    }

    pub fn binding_digest(&self) -> Result<[u8; 32], CoordinatorStabilityFenceError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_SUBJECT);
        frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame(&mut hasher, ADMISSION_SUBJECT_PROFILE.as_bytes());
        frame(&mut hasher, &self.subject_digest);
        frame(&mut hasher, self.subject_profile.as_bytes());
        frame(&mut hasher, &self.attempt_nonce);
        frame(&mut hasher, self.attempt_ref.as_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Non-deserializable proof that the exact coordinator deployment matched both
/// before and after one named admission attempt interval.
///
/// This is a stability observation theorem, not a mutual-exclusion/atomicity lock
/// and not permission to perform an effect.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoordinatorStabilityFence {
    pre_composition: QualifiedCoordinatorDeploymentComposition,
    post_match: MatchedCoordinatorDeployment,
    admission_subject_digest: [u8; 32],
    admission_subject_profile: String,
    pre_observed_at_ms: u64,
    post_observed_at_ms: u64,
    fence_digest: [u8; 32],
    fence_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoordinatorStabilityFence {
    pub fn pre_composition(&self) -> &QualifiedCoordinatorDeploymentComposition {
        &self.pre_composition
    }

    pub fn post_match(&self) -> &MatchedCoordinatorDeployment {
        &self.post_match
    }

    pub fn admission_subject_digest(&self) -> [u8; 32] {
        self.admission_subject_digest
    }

    pub fn admission_subject_profile(&self) -> &str {
        &self.admission_subject_profile
    }

    pub fn pre_observed_at_ms(&self) -> u64 {
        self.pre_observed_at_ms
    }

    pub fn post_observed_at_ms(&self) -> u64 {
        self.post_observed_at_ms
    }

    pub fn fence_digest(&self) -> [u8; 32] {
        self.fence_digest
    }

    pub fn fence_profile(&self) -> &str {
        &self.fence_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }
}

pub fn qualify_coordinator_stability_fence(
    pre_composition: QualifiedCoordinatorDeploymentComposition,
    subject: &CoordinatorAdmissionSubject,
    post_observation: &ObservedCoordinatorDeployment,
    now_ms: u64,
) -> Result<QualifiedCoordinatorStabilityFence, CoordinatorStabilityFenceError> {
    subject.validate()?;

    let pre_match = pre_composition.matched_deployment();
    let pre_observed_at_ms = pre_match.observed_at_ms();
    if post_observation.observed_at_ms <= pre_observed_at_ms {
        return Err(CoordinatorStabilityFenceError::PostObservationNotLater);
    }
    if post_observation.observed_at_ms > pre_composition.valid_until_ms() {
        return Err(CoordinatorStabilityFenceError::PostObservationOutsidePreWindow);
    }
    if now_ms == 0
        || pre_composition.verified_at_ms() > now_ms
        || pre_composition.valid_until_ms() <= now_ms
    {
        return Err(CoordinatorStabilityFenceError::PreCompositionNotLive);
    }

    let current_release = pre_composition.current_release();
    let required = current_release
        .release()
        .required_deployment_for_target_agent(
            pre_composition.target_agent_pub_key_raw_39().to_vec(),
        )
        .map_err(|error| CoordinatorStabilityFenceError::ReleaseSpecialization(error.to_string()))?;

    let required_digest = required
        .requirement_digest()
        .map_err(CoordinatorStabilityFenceError::DeploymentMatch)?;
    if required_digest != pre_composition.required_deployment_digest()
        || pre_composition.required_deployment_profile() != REQUIREMENT_PROFILE
    {
        return Err(CoordinatorStabilityFenceError::RequiredDeploymentChanged);
    }

    let post_match = match_required_coordinator_deployment(&required, post_observation, now_ms)
        .map_err(CoordinatorStabilityFenceError::DeploymentMatch)?;

    if post_match.dna_hash_raw_39() != pre_composition.target_dna_hash_raw_39()
        || post_match.agent_pub_key_raw_39() != pre_composition.target_agent_pub_key_raw_39()
    {
        return Err(CoordinatorStabilityFenceError::PostCellMismatch);
    }
    if post_match.requirement_digest() != pre_composition.required_deployment_digest()
        || post_match.requirement_profile() != pre_composition.required_deployment_profile()
    {
        return Err(CoordinatorStabilityFenceError::RequiredDeploymentChanged);
    }

    let verified_at_ms = pre_composition
        .verified_at_ms()
        .max(post_match.observed_at_ms());
    let valid_until_ms = pre_composition
        .valid_until_ms()
        .min(post_match.valid_until_ms());
    if verified_at_ms > now_ms || valid_until_ms <= now_ms || valid_until_ms <= verified_at_ms {
        return Err(CoordinatorStabilityFenceError::EmptyFenceWindow);
    }

    let admission_subject_digest = subject.binding_digest()?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_FENCE);
    frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, FENCE_PROFILE.as_bytes());
    frame(&mut hasher, &pre_composition.composition_digest());
    frame(
        &mut hasher,
        pre_composition.composition_profile().as_bytes(),
    );
    frame(&mut hasher, &post_match.match_digest());
    frame(&mut hasher, post_match.match_profile().as_bytes());
    frame(&mut hasher, &admission_subject_digest);
    frame(&mut hasher, ADMISSION_SUBJECT_PROFILE.as_bytes());
    frame(&mut hasher, &pre_observed_at_ms.to_le_bytes());
    frame(&mut hasher, &post_match.observed_at_ms().to_le_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    let fence_digest = *hasher.finalize().as_bytes();

    Ok(QualifiedCoordinatorStabilityFence {
        pre_composition,
        post_match,
        admission_subject_digest,
        admission_subject_profile: ADMISSION_SUBJECT_PROFILE.into(),
        pre_observed_at_ms,
        post_observed_at_ms: post_observation.observed_at_ms,
        fence_digest,
        fence_profile: FENCE_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn validate_digest(
    value: &[u8; 32],
    field: &'static str,
) -> Result<(), CoordinatorStabilityFenceError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorStabilityFenceError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), CoordinatorStabilityFenceError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(CoordinatorStabilityFenceError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorStabilityFenceError {
    WrongProtocol,
    InvalidDigest(&'static str),
    InvalidText(&'static str),
    PreCompositionNotLive,
    PostObservationNotLater,
    PostObservationOutsidePreWindow,
    ReleaseSpecialization(String),
    DeploymentMatch(CoordinatorDeploymentError),
    RequiredDeploymentChanged,
    PostCellMismatch,
    EmptyFenceWindow,
}

impl fmt::Display for CoordinatorStabilityFenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator stability-fence protocol"),
            Self::InvalidDigest(field) => write!(f, "invalid {field}"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::PreCompositionNotLive => {
                write!(f, "pre-deployment composition is not live at fence qualification")
            }
            Self::PostObservationNotLater => {
                write!(f, "post observation must be strictly later than pre observation")
            }
            Self::PostObservationOutsidePreWindow => write!(
                f,
                "post observation occurred outside the pre-composition evidence window"
            ),
            Self::ReleaseSpecialization(error) => {
                write!(f, "current release specialization failed: {error}")
            }
            Self::DeploymentMatch(error) => {
                write!(f, "post coordinator deployment match denied: {error}")
            }
            Self::RequiredDeploymentChanged => {
                write!(f, "post fence no longer names the exact pre required deployment")
            }
            Self::PostCellMismatch => {
                write!(f, "post observation does not match the exact pre target CellId")
            }
            Self::EmptyFenceWindow => {
                write!(f, "coordinator stability fence has no live evidence window")
            }
        }
    }
}

impl std::error::Error for CoordinatorStabilityFenceError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_coordinator_deployment::{
        CoordinatorCodeIdentity, CONDUCTOR_ADMIN_SOURCE_PROFILE,
        PROTOCOL_VERSION as DEPLOYMENT_PROTOCOL_VERSION,
    };
    use mycelix_authority_coordinator_deployment_composer::{
        qualify_current_release_against_observed_deployment, TargetCellSelection,
        PROTOCOL_VERSION as COMPOSER_PROTOCOL_VERSION,
    };
    use mycelix_authority_coordinator_release::{
        qualify_coordinator_release, CoordinatorReleaseManifest,
        VerifiedCoordinatorReleaseSignatureProof, MANIFEST_PROFILE,
        PROTOCOL_VERSION as RELEASE_PROTOCOL_VERSION, SIGNATURE_PROOF_PROTOCOL,
    };
    use mycelix_authority_coordinator_release_currentness::{
        qualify_current_coordinator_release, CoordinatorReleaseStatus,
        QualifiedCurrentCoordinatorRelease, VerifiedCoordinatorReleaseStatusAtHeadProof,
        VerifiedCurrentReleaseRegistryHeadProof, PROTOCOL_VERSION as CURRENTNESS_PROTOCOL_VERSION,
        REGISTRY_HEAD_PROFILE, STATUS_RECORD_PROFILE,
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
            protocol_version: COMPOSER_PROTOCOL_VERSION.into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            selection_ref: "target-cell:governance:primary".into(),
        }
    }

    fn observation(at: u64, until: u64) -> ObservedCoordinatorDeployment {
        ObservedCoordinatorDeployment {
            protocol_version: DEPLOYMENT_PROTOCOL_VERSION.into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            coordinators: vec![coordinator("authority_current_freshness_verifier", 1)],
            source_profile: CONDUCTOR_ADMIN_SOURCE_PROFILE.into(),
            source_ref: "admin-websocket:loopback:cell:test".into(),
            observed_at_ms: at,
            valid_until_ms: until,
        }
    }

    fn pre_composition() -> QualifiedCoordinatorDeploymentComposition {
        qualify_current_release_against_observed_deployment(
            current_release(),
            &target(),
            &observation(190, 300),
            200,
        )
        .unwrap()
    }

    fn subject(nonce: u8) -> CoordinatorAdmissionSubject {
        CoordinatorAdmissionSubject {
            protocol_version: PROTOCOL_VERSION.into(),
            subject_digest: d(21),
            subject_profile: "mycelix-effect-admission-subject-v1".into(),
            attempt_nonce: d(nonce),
            attempt_ref: format!("admission-attempt:{nonce}"),
        }
    }

    #[test]
    fn strictly_later_matching_observation_qualifies() {
        let fence = qualify_coordinator_stability_fence(
            pre_composition(),
            &subject(22),
            &observation(210, 310),
            220,
        )
        .unwrap();
        assert_eq!(fence.pre_observed_at_ms(), 190);
        assert_eq!(fence.post_observed_at_ms(), 210);
        assert_eq!(fence.valid_until_ms(), 300);
        assert_ne!(fence.fence_digest(), [0; 32]);
    }

    #[test]
    fn same_or_older_observation_denies() {
        for at in [189, 190] {
            assert_eq!(
                qualify_coordinator_stability_fence(
                    pre_composition(),
                    &subject(22),
                    &observation(at, 300),
                    200,
                )
                .unwrap_err(),
                CoordinatorStabilityFenceError::PostObservationNotLater
            );
        }
    }

    #[test]
    fn post_observation_outside_pre_window_denies_explicitly() {
        assert_eq!(
            qualify_coordinator_stability_fence(
                pre_composition(),
                &subject(22),
                &observation(301, 350),
                302,
            )
            .unwrap_err(),
            CoordinatorStabilityFenceError::PostObservationOutsidePreWindow
        );
    }

    #[test]
    fn changed_coordinator_or_cell_denies() {
        let mut changed_code = observation(210, 310);
        changed_code.coordinators[0].wasm_hash_raw_39 = h(99);
        assert!(matches!(
            qualify_coordinator_stability_fence(
                pre_composition(),
                &subject(22),
                &changed_code,
                220,
            ),
            Err(CoordinatorStabilityFenceError::DeploymentMatch(_))
        ));

        let mut changed_cell = observation(210, 310);
        changed_cell.agent_pub_key_raw_39 = h(7);
        assert!(matches!(
            qualify_coordinator_stability_fence(
                pre_composition(),
                &subject(22),
                &changed_cell,
                220,
            ),
            Err(CoordinatorStabilityFenceError::DeploymentMatch(_))
        ));
    }

    #[test]
    fn admission_attempt_nonce_changes_fence_identity() {
        let first = qualify_coordinator_stability_fence(
            pre_composition(),
            &subject(22),
            &observation(210, 310),
            220,
        )
        .unwrap();
        let second = qualify_coordinator_stability_fence(
            pre_composition(),
            &subject(23),
            &observation(210, 310),
            220,
        )
        .unwrap();
        assert_ne!(
            first.admission_subject_digest(),
            second.admission_subject_digest()
        );
        assert_ne!(first.fence_digest(), second.fence_digest());
    }

    #[test]
    fn shortest_pre_or_post_lease_caps_fence() {
        let fence = qualify_coordinator_stability_fence(
            pre_composition(),
            &subject(22),
            &observation(210, 230),
            220,
        )
        .unwrap();
        assert_eq!(fence.valid_until_ms(), 230);
    }
}
